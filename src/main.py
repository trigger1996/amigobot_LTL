#!/usr/bin/env python
#encoding=utf-8

import rospy
import math
from geometry_msgs.msg import Twist
from bot import amigobot, amigobot_xyControl

def main():
    rospy.init_node('motion_primitive_test', anonymous=False)

    bot_1 = amigobot_xyControl(name='amigobot_1')
    bot_2 = amigobot_xyControl(name='amigobot_2')
    rate = rospy.Rate(25)	# 5Hz

    rospy.sleep(3)

    #bot_1.turn_90_ccw()
    #bot_2.turn_90_cw()
    #bot_1.set_vel(0, math.pi / 2)
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
