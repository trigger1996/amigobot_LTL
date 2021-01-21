#!/usr/bin/env python
#encoding=utf-8

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion

class amigobot:
    def __init__(self, name='RosAria'):
        self.name = name

        self.twist_pub = rospy.Publisher('/' + name + '/cmd_vel', Twist, queue_size = 10)
        self.pose_sub  = rospy.Subscriber('/' + name + '/odom', Odometry, self.odom_cb)

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
            self.set_vel(0, -math.pi / 2)
            if self.yaw <= self.yaw_desired:
                self.current_motion = 'wait'

        elif self.current_motion == 'left_turn':
            self.set_vel(0, math.pi / 2)
            if self.yaw >= self.yaw_desired:
                self.current_motion = 'wait'
        
        elif self.current_motion == 'forward':
            self.dist_curr = math.sqrt((self.x - self.x_start)**2 + (self.y - self.y_start)**2)
            
            self.set_vel(0.48, 0)
            if self.dist_curr >= self.dist_desired:
                self.x_start = self.x
                self.y_start = self.y
                self.dist_curr = 0
                self.dist_desired = 0

                self.current_motion = 'wait'

        elif self.current_motion == 'back':
            self.dist_curr = math.sqrt((self.x - self.x_start)**2 + (self.y - self.y_start)**2)

            self.set_vel(-0.48, 0)
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

def main():
    rospy.init_node('motion_primitive_test', anonymous=False)

    bot_1 = amigobot(name='amigobot_1')
    bot_2 = amigobot(name='amigobot_2')
    rate = rospy.Rate(25)	# 5Hz

    rospy.sleep(3)

    #bot_1.turn_90_ccw()
    #bot_2.turn_90_cw()
    #bot_1.set_vel(0, math.pi / 2)
    bot_1.turn_cw(45)
    '''
    for j in range(0, 8):
	    for i in range(0, 75):
            bot_1.single_forward()
            rate.sleep()

	    for i in range(0, 25):
            bot_1.single_turn()
            rate.sleep()
    '''

    while not rospy.is_shutdown():
        #bot_1.set_vel(0, 0.2)
        #bot_2.set_vel(0, -0.2)

        rate.sleep()


    print(233)

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass
