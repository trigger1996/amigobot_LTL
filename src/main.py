#!/usr/bin/env python
#encoding=utf-8
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, fabs
from tf.transformations import euler_from_quaternion
import numpy as np

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
        self.waypt = []             # next target list [[x, y, yaw]]

        self.is_all_done = False

        # v-w controller target
        self.yaw_setpoint = 0
        self.dist_setpoint = 0

        self.yaw_setpt_threshold  = 2          # deg, ref: 5
        self.dist_setpt_threshold = 0.05

        # yaw PI controller
        self.yaw_kp = 2.25
        self.yaw_ki = 0.15
        self.yaw_increment  = 0
        self.yaw_inc_max = 0.15
        self.u_yaw_max = 180         # deg/s



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
        if self.is_vertex_arrived(self.target_x, self.target_y):
            # send a break
            self.update_target_vel(0, 0)            

            # no next points
            if self.waypt.__len__() == 0:
                self.is_all_done = True
            else:
                [self.target_x, self.target_y, self.target_yaw] = self.waypt.pop(0)

                # if waypoint is the same, wait

                self.yaw_setpoint  = atan2(self.target_y - self.y, self.target_x - self.x) * 180 / pi
                self.dist_setpoint = sqrt((self.target_y - self.y)**2 + (self.target_x - self.x)**2)
        else:
            if fabs(self.yaw_setpoint - self.yaw) >= self.yaw_setpt_threshold:
                # PI control w saturation
                yaw_err = self.yaw_setpoint - self.yaw
                while yaw_err >= 180.:
                    yaw_err -= 360.
                while yaw_err < -180.:
                    yaw_err += 360.
                
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

                self.update_target_vel(0, u_yaw * pi / 180)

                print(self.yaw_kp * yaw_err, yaw_err, u_yaw)
                print(fabs(self.yaw_setpoint - self.yaw))

            else:
                # for testing, send a break
                self.update_target_vel(0, 0)      



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

    rate = rospy.Rate(50)	# 5Hz

    rospy.sleep(5)

    bot_1 = turtlebot(name='amigobot_1', model=None)
    
    bot_1.add_waypoint(-3, 0)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass