#!/usr/bin/env python
#encoding=utf-8

import rospy
from geometry_msgs.msg import Twist
from bot import amigobot

def main():
    rospy.init_node('motion_primitive_test', anonymous=False)

    bot_1 = amigobot(name='amigobot_1')
    bot_2 = amigobot(name='amigobot_2')
    rate = rospy.Rate(25)	# 5Hz

    rospy.sleep(1)

    for j in range(0, 8):
	    for i in range(0, 75):
		bot_1.single_forward()
		rate.sleep()

	    for i in range(0, 25):
		bot_1.single_turn()
		rate.sleep()

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
