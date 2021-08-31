#!/usr/bin/env python
#encoding=utf-8
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np

class turtlebot(object):
    def __init__(self, name):
        self.name = name

        self.twist_pub = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size = 1)
        self.pose_sub  = rospy.Subscriber('/' + self.name + '/odom', Odometry, self.odom_cb)

        self.x = 0
        self.y = 0
        self.yaw = 0

        self.vx = 0
        self.wz = 0
    

    def odom_cb(self, data):
        # about 10Hz
        (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x, 
                                                    data.pose.pose.orientation.y, 
                                                    data.pose.pose.orientation.z, 
                                                    data.pose.pose.orientation.w])
        self.yaw = yaw * 180. / pi
        self.x   = data.pose.pose.position.x
        self.y   = data.pose.pose.position.y


        self.update_target_vel()

    def update_target_vel(self):
        move_cmd = Twist()
        move_cmd.linear.x = self.vx
        move_cmd.angular.z =self.wz
        self.twist_pub.publish(move_cmd)


def main():
    rospy.init_node('ijrr2013_ca_improv', anonymous=False)

    rate = rospy.Rate(50)	# 5Hz

    rospy.sleep(5)

    bot_1 = turtlebot(name='amigobot_1')
    bot_1.vx = 0.5
    bot_1.wz = pi / 16

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass