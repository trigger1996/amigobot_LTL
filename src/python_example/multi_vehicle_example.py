#!/usr/bin/env python
#encoding=utf-8

import rospy
from geometry_msgs.msg import Twist

class amigobot:
    def __init__(self, name='amigobot'):
        self.name = name

        self.twist_pub = rospy.Publisher('/' + name + '/cmd_vel', Twist, queue_size = 10)

        self.vx = 0
        self.wz = 0

    def set_vel(self, vx, wz):
        self.vx = vx
        self.wz = wz

        move_cmd = Twist()
        move_cmd.linear.x = self.vx
        move_cmd.angular.z =self.wz
        self.twist_pub.publish(move_cmd)


def main():
    rospy.init_node('motion_primitive_test', anonymous=False)

    bot_1 = amigobot(name='amigobot_1')
    bot_2 = amigobot(name='amigobot_2')
    rate = rospy.Rate(5)	# 5Hz

    while not rospy.is_shutdown():
        bot_1.set_vel(0.2, 0.2)
        bot_2.set_vel(0.2, -0.2)

        rospy.sleep(1)


    print(233)

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass
