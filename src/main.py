#!/usr/bin/env python
#encoding=utf-8
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, fabs
from tf.transformations import euler_from_quaternion
import numpy as np

global ros_rate
global time_to_wait
ros_rate = 50
time_to_wait = 4        # seconds

class turtlebot(object):
    def __init__(self, name, model, x = 0, y = 0, yaw = 0):
        self.name = name

        self.twist_pub = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size = 1)
        self.pose_sub  = rospy.Subscriber('/' + self.name + '/odom', Odometry, self.odom_cb)

        self.x = x
        self.y = y
        self.yaw = yaw

        self.vx = 0
        self.wz = 0

        # waypoint varibles
        self.target_x = self.x      # current target
        self.target_y = self.y
        self.target_yaw = self.yaw
        self.target_x_last = None
        self.target_y_last = None
        self.target_yaw_last = None
        self.waypt = []             # next target list [[x, y, yaw]]

        self.is_all_done = False

        # v-w controller target
        self.yaw_setpoint = 0
        #self.dist_setpoint = 0

        self.yaw_setpt_threshold  = 0.5          # deg, ref: 5
        self.dist_setpt_threshold = 0.15

        # yaw PI controller
        self.yaw_kp = 2.25
        self.yaw_ki = 0.15
        self.yaw_increment  = 0
        self.yaw_inc_max = 0.15
        self.u_yaw_max = 180         # deg/s

        # dist PI controller
        self.dist_kp = 0.3
        self.dist_ki = 0.15
        self.dist_increment  = 0
        self.dist_inc_max = 0.15
        self.u_dist_max = 0.25        # m/s, maximum speed with no slide in startup: 0.3 (about)

        self.is_steer_completed = False
        self.is_wait = False          # symbol for waiting
        self.wait_index = 0


    def odom_cb(self, data):
        # about 10Hz
        (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x, 
                                                    data.pose.pose.orientation.y, 
                                                    data.pose.pose.orientation.z, 
                                                    data.pose.pose.orientation.w])
        self.yaw = yaw * 180. / pi
        self.x   = data.pose.pose.position.x
        self.y   = data.pose.pose.position.y

        self.motion_control()

    def motion_control(self):
        # first check if arrived next waypoint
        if self.is_vertex_arrived(self.target_x, self.target_y, self.dist_setpt_threshold):
            # send a break
            self.update_target_vel(0, 0)
            self.is_steer_completed = False

            # no next points
            if self.waypt.__len__() == 0:
                self.is_all_done = True
            
            # next point exists
            elif self.is_wait == False:
                self.target_x_last   = self.target_x
                self.target_y_last   = self.target_y
                self.target_yaw_last = self.target_yaw

                [self.target_x, self.target_y, self.target_yaw] = self.waypt.pop(0)

                self.yaw_setpoint  = atan2(self.target_y - self.y, self.target_x - self.x) * 180 / pi
                #self.dist_setpoint = sqrt((self.target_y - self.y)**2 + (self.target_x - self.x)**2)

                # if waypoint is the same, wait
                if self.target_x_last  == self.target_x and self.target_y_last == self.target_y:
                    self.wait_index = 0
                    self.is_wait = True

            # waiting
            elif self.is_wait == True and self.wait_index <= ros_rate * time_to_wait:
                self.wait_index += 1
                if self.wait_index >= ros_rate * time_to_wait:
                    self.wait_index = 0
                    self.is_wait = False

        #
        else:

            # calculate errors
            yaw_err = atan2(self.target_y - self.y, self.target_x - self.x) * 180 / pi - self.yaw
            dist_err = sqrt((self.target_y - self.y)**2 + (self.target_x - self.x)**2)            
            while yaw_err >= 180.:
                yaw_err -= 360.
            while yaw_err < -180.:
                yaw_err += 360.

            # turn first
            if fabs(yaw_err) >= self.yaw_setpt_threshold and self.is_steer_completed == False:
                # PI control w saturation               
                self.yaw_increment += yaw_err
                if self.yaw_increment > self.yaw_inc_max:
                    self.yaw_increment = self.yaw_inc_max
                elif self.yaw_increment < -self.yaw_inc_max:
                    self.yaw_increment = -self.yaw_inc_max

                u_yaw = self.yaw_kp * yaw_err + self.yaw_ki * self.yaw_increment
                if u_yaw > self.u_yaw_max:
                    u_yaw = self.u_yaw_max
                elif u_yaw < -self.u_yaw_max:
                    u_yaw = -self.u_yaw_max

                if fabs(yaw_err) <= self.yaw_setpt_threshold + 0.1:         # for easy to stop steering
                    self.is_steer_completed = True

                self.update_target_vel(0, u_yaw * pi / 180)

            else:
                # PI control w saturation
                # dist
                self.dist_increment += dist_err
                if self.dist_increment > self.dist_inc_max:
                    self.dist_increment = self.dist_inc_max
                elif self.dist_increment < -self.dist_inc_max:
                    self.dist_increment = -self.dist_inc_max

                u_dist = self.dist_kp * dist_err + self.dist_ki * self.dist_increment
                if u_dist > self.u_dist_max:
                    u_dist = self.u_dist_max
                elif u_dist < -self.u_dist_max:
                    u_dist = -self.u_dist_max

                # yaw P controller
                u_yaw = self.yaw_kp * yaw_err
                if u_yaw > self.u_yaw_max:
                    u_yaw = self.u_yaw_max
                elif u_yaw < -self.u_yaw_max:
                    u_yaw = -self.u_yaw_max

                #
                self.update_target_vel(u_dist, u_yaw * pi / 180 * 0.066)
                #self.update_target_vel(u_dist, 0)


    def update_target_vel(self, v, w):
        self.vx = v
        self.wz = w

        move_cmd = Twist()
        move_cmd.linear.x = self.vx
        move_cmd.angular.z =self.wz
        self.twist_pub.publish(move_cmd)

    def is_vertex_arrived(self, x, y, threshold = 0.05):
        dist = sqrt((self.x - x) ** 2 + (self.y - y) ** 2)
        if dist <= threshold:
            return True
        else:
            return False

    def add_waypoint(self, x, y, yaw = 0):
        self.waypt.append([x, y, yaw])
        self.is_all_done = False

        # if there is no waypoint for the current target, add it
        if self.waypt.__len__() == 0:
            [self.target_x, self.target_y, self.target_yaw] = self.waypt.pop(0)
            self.yaw_setpoint  = atan2(self.target_y - self.y, self.target_x - self.x)
            self.dist_setpoint = sqrt((self.target_y - self.y)**2 + (self.target_x - self.x)**2)        

def main():
    rospy.init_node('ijrr2013_ca_improv', anonymous=False)

    rate = rospy.Rate(ros_rate)	# 5Hz

    rospy.sleep(5)

    bot_1 = turtlebot(name='amigobot_1', model=None)
    
    bot_1.add_waypoint( 0,  0)    
    bot_1.add_waypoint(-3,  0)
    bot_1.add_waypoint(-3, -3)
    bot_1.add_waypoint(-3, -3)    
    bot_1.add_waypoint( 0, -3)
    bot_1.add_waypoint( 0,  0)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass