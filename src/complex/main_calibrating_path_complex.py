#!/usr/bin/env python
#encoding=utf-8

import sys
sys.path.append("/home/ubuntu484/catkin_ws/src/amigobot_LTL/src/LOMAP_Custom/")               # root path: amigobot_LTL/src
sys.path.append("/home/ubuntu484/catkin_ws/src/amigobot_LTL/src/") 

import rospy
import bot_libs.bot as bot
import bot_libs.bot_ts as bot_ts

from math import pi
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler

global time_to_wait
time_to_wait = 10        # seconds

def main():
    rospy.init_node('ijrr2013_ca_improv', anonymous=False)

    rate = rospy.Rate(60)	# 60Hz

    rospy.sleep(8)

 
    bot_1 = bot_ts.turtlebot_TS(name='amigobot_1', yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/complex/robot_1.yaml',      # /home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/
                                                   map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/complex/map_controllable.yaml',
                                                   time_to_wait = time_to_wait)
    
    bot_1.add_waypoint_from_waypt_list('u1')
    bot_1.add_waypoint_from_waypt_list('u2')
    bot_1.add_waypoint_from_waypt_list('11')
    bot_1.add_waypoint_from_waypt_list('12')
    bot_1.add_waypoint_from_waypt_list('13')
    bot_1.add_waypoint_from_waypt_list('g1')
    bot_1.add_waypoint_from_waypt_list('13')
    bot_1.add_waypoint_from_waypt_list('11')
    bot_1.add_waypoint_from_waypt_list('12')
    bot_1.add_waypoint_from_waypt_list('14')
    bot_1.add_waypoint_from_waypt_list('15')
    bot_1.add_waypoint_from_waypt_list('g2')
    bot_1.add_waypoint_from_waypt_list('15')
    bot_1.add_waypoint_from_waypt_list('16')
    bot_1.add_waypoint_from_waypt_list('u2')    
    bot_1.add_waypoint_from_waypt_list('u1')

    bot_1.add_waypoint_from_waypt_list('u1')
    bot_1.add_waypoint_from_waypt_list('21')
    bot_1.add_waypoint_from_waypt_list('26')
    bot_1.add_waypoint_from_waypt_list('25')
    bot_1.add_waypoint_from_waypt_list('21')
    bot_1.add_waypoint_from_waypt_list('25')
    bot_1.add_waypoint_from_waypt_list('24')
    bot_1.add_waypoint_from_waypt_list('g3')
    bot_1.add_waypoint_from_waypt_list('24')
    bot_1.add_waypoint_from_waypt_list('23')
    bot_1.add_waypoint_from_waypt_list('22')
    bot_1.add_waypoint_from_waypt_list('u1')

    while not rospy.is_shutdown():

        rate.sleep()

    print("Finished!")

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass