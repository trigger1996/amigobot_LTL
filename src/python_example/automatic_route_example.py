#!/usr/bin/env python
#encoding=utf-8

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion

class amigobot(object):
    def __init__(self, name='RosAria'):
        self.name = name

        self.twist_pub = rospy.Publisher('/' + name + '/cmd_vel', Twist, queue_size = 10)
        self.pose_sub  = rospy.Subscriber('/' + name + '/pose', Odometry, self.odom_cb)

        self.vx = 0
        self.wz = 0

        self.x = 0
        self.y = 0
        self.yaw = 0

        self.x_start = self.x
        self.y_start = self.y
        self.dist_desired = 0
        self.dist_curr = 0
        self.yaw_desired = 0

        self.basic_motion = ['wait', 'forward', 'back', 'left_turn', 'right_turn']
        self.current_motion = 'wait'


    def odom_cb(self, data):
        (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x, 
                                                    data.pose.pose.orientation.y, 
                                                    data.pose.pose.orientation.z, 
                                                    data.pose.pose.orientation.w])
        self.yaw = yaw * 180. / math.pi
        self.x   = data.pose.pose.position.x
        self.y   = data.pose.pose.position.y

        # set vel
        if self.current_motion == 'wait':
            self.set_vel(0, 0)

        elif self.current_motion == 'right_turn':
            self.set_vel(0, -math.pi / 4)
            if self.yaw <= self.yaw_desired:
                self.current_motion = 'wait'

        elif self.current_motion == 'left_turn':
            self.set_vel(0, math.pi / 4)
            if self.yaw >= self.yaw_desired:
                self.current_motion = 'wait'
        
        elif self.current_motion == 'forward':
            self.dist_curr = math.sqrt((self.x - self.x_start)**2 + (self.y - self.y_start)**2)
            
            self.set_vel(0.33, 0)                       # 0.48
            if self.dist_curr >= self.dist_desired:
                self.x_start = self.x
                self.y_start = self.y
                self.dist_curr = 0
                self.dist_desired = 0

                self.current_motion = 'wait'

        elif self.current_motion == 'back':
            self.dist_curr = math.sqrt((self.x - self.x_start)**2 + (self.y - self.y_start)**2)

            self.set_vel(-0.33, 0)                       # -0.48
            if self.dist_curr >= self.dist_desired:
                self.x_start = self.x
                self.y_start = self.y
                self.dist_curr = 0
                self.dist_desired = 0

                self.current_motion = 'wait'

        self.update_target_vel()


    def update_target_vel(self):
        move_cmd = Twist()
        move_cmd.linear.x = self.vx
        move_cmd.angular.z =self.wz
        self.twist_pub.publish(move_cmd)

    def set_vel_w_update(self, vel, yaw):
        self.vx = vel
        self.wz = yaw

        move_cmd = Twist()
        move_cmd.linear.x = self.vx
        move_cmd.angular.z =self.wz
        self.twist_pub.publish(move_cmd)

    def set_vel(self, vel, yaw):
        self.vx = vel
        self.wz = yaw

    def turn_90_cw(self):
        self.yaw_desired = self.yaw - 90.
        self.current_motion = 'right_turn'

    def turn_90_ccw(self):
        self.yaw_desired = self.yaw + 90.
        self.current_motion = 'left_turn'

    def turn_cw(self, target_yaw):
        # in degree
        self.yaw_desired = self.yaw - target_yaw
        self.current_motion = 'right_turn'

    def turn_ccw(self, target_yaw):
        # in degree
        self.yaw_desired = self.yaw + target_yaw
        self.current_motion = 'left_turn'        

    def forward(self, dist):
        self.x_start = self.x
        self.y_start = self.y
        self.dist_desired = math.fabs(dist)
        self.current_motion = 'forward'

    def back(self, dist):
        self.x_start = self.x
        self.y_start = self.y
        self.dist_desired = math.fabs(dist)
        self.current_motion = 'back'

    # motion primitives with costs #RosAria
    def mp_left_up(dist):
        self.set_vel(-0.5, 0)

class amigobot_xyControl(amigobot):
    def __init__(self, name='RosAria'):
        super(amigobot_xyControl, self).__init__(name)


        self.route_list = []        # [[x1, y1], ..., [xn, yn]], "FLU" frame from start

        self.x_tgt = self.x 
        self.y_tgt = self.y
        self.yaw_to_turn = 0

        self.is_all_done = False    # added for while-loop

    def odom_cb(self, data):
        # update xy

        # target arrived
        if self.is_vertex_arrived(self.x_tgt, self.y_tgt):
            # with no next target
            if self.route_list.__len__() == 0:
                self.current_motion == 'wait'
                self.is_all_done = True
            # update next target
            else:
                [self.x_tgt, self.y_tgt] = self.route_list.pop(0)

                # update target parameters
                self.yaw_to_turn  = math.atan2(self.y_tgt - self.y, self.x_tgt - self.x) * 180. / math.pi
                self.yaw_to_turn  = self.yaw_to_turn - self.yaw
                self.dist_desired = math.sqrt((self.x_tgt - self.x)**2 + (self.y_tgt - self.y)**2)

        # target_not_arrived
        else:
            # first, align robot with target
            if self.is_angle_arrived() == False:
                if self.yaw_to_turn <= 0:
                    self.turn_cw(self.yaw_to_turn)
                else:
                    self.turn_ccw(self.yaw_to_turn)
            # second, go
            else:
                #self.forward(self.dist_desired)
                self.current_motion = 'forward'


        # update original data
        super(amigobot_xyControl, self).odom_cb(data)

    def is_angle_arrived(self, yaw_err = 20):
        #in degree
        target_yaw = math.atan2(self.y_tgt - self.y, self.x_tgt - self.x) * 180. / math.pi
        if math.fabs(self.yaw - target_yaw) <= yaw_err:
            return True
        else:
            return False
    
    def is_vertex_arrived(self, x, y, err = 0.5):
        dist_err = math.sqrt((x -self.x)**2 + (y - self.y)**2)
        if math.fabs(dist_err) <= err:
            return True
        else:
            return False

    # in FLU frame for start point
    def add_waypoint(self, x, y):
        self.route_list.append([x, y])
        self.is_all_done = False

    def clear_waypoint(self):
        self.route_list.clear()

def main():
    rospy.init_node('motion_primitive_test', anonymous=False)

    bot_1 = amigobot_xyControl(name='amigobot_1')
    #bot_2 = amigobot_xyControl(name='amigobot_2')
    rate = rospy.Rate(25)	# 5Hz

    rospy.sleep(3)

    bot_1.add_waypoint(0, 1)
    bot_1.add_waypoint(1, 1)
    bot_1.add_waypoint(1, 0)
    bot_1.add_waypoint(0, 0)

    while not rospy.is_shutdown():
        if bot_1.is_all_done == True:
            bot_1.add_waypoint(1, 1)

        rate.sleep()


    print(233)

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass
