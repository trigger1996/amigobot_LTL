#!/usr/bin/env python
#encoding=utf-8

import sys
import rospy
import math
from geometry_msgs.msg import Twist
from amigobot_lib.bot import amigobot, amigobot_xyControl
from amigobot_lib.bot_ts import amigobot_TS

def main():
    rospy.init_node('motion_primitive_test', anonymous=False)

    bot_1 = amigobot_TS(name='amigobot_1', yaml_file=sys.path[0] + '/../model/ijrr_2013_improv/robot_1.yaml',
                                           map_file =sys.path[0] + '/../model/ijrr_2013_improv/map.yaml')
    bot_2 = amigobot_TS(name='amigobot_2', yaml_file=sys.path[0] + '/../model/ijrr_2013_improv/robot_3.yaml',
                                           map_file =sys.path[0] + '/../model/ijrr_2013_improv/map.yaml')
    rate = rospy.Rate(25)	# 5Hz

    rospy.sleep(3)

    bot_1.add_waypoint_from_waypt_list('28')
    bot_1.add_waypoint_from_waypt_list('1')
    bot_2.add_waypoint_from_waypt_list('u1')
    bot_2.add_waypoint_from_waypt_list('1')
    bot_2.add_waypoint_from_waypt_list('g1')

    #bot_1.turn_90_ccw()
    #bot_2.turn_90_cw()
    #bot_1.set_vel(0, math.pi / 2)
    #bot_1.add_waypoint(0, 1)
    #bot_1.add_waypoint(1, 1)
    #bot_1.add_waypoint(1, 0)
    #bot_1.add_waypoint(0, 0)

    while not rospy.is_shutdown():
        if bot_1.is_all_done == True:
            bot_1.add_waypoint_from_waypt_list('28')
            bot_1.add_waypoint_from_waypt_list('1')

        if bot_2.is_all_done == True:
            bot_2.add_waypoint_from_waypt_list('u1')
            bot_2.add_waypoint_from_waypt_list('1')
            bot_2.add_waypoint_from_waypt_list('g1')

        rate.sleep()


    print(233)

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass
