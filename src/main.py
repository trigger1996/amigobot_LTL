#!/usr/bin/env python
#encoding=utf-8

import sys
sys.path.append("/home/ghost/catkin_ws_ros/src/amigobot_LTL/src/LOMAP_Custom/")               # root path: amigobot_LTL/src

import rospy
import bot_libs.bot as bot
import bot_libs.bot_ts as bot_ts


def main():
    rospy.init_node('ijrr2013_ca_improv', anonymous=False)

    rate = rospy.Rate(50)	# 50Hz

    rospy.sleep(5)

    bot_1 = bot_ts.turtlebot_TS(name='amigobot_1', model=None, yaml_file='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/robot_1.yaml',
                                                               map_file ='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/map.yaml')
    bot_2 = bot_ts.turtlebot_TS(name='amigobot_2', model=None, yaml_file='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/robot_2.yaml',
                                                               map_file ='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/map.yaml')
    bot_3 = bot_ts.turtlebot_TS(name='amigobot_3', model=None, yaml_file='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/robot_3.yaml',
                                                               map_file ='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/map.yaml')                                                               


    bot_1.add_waypoint_from_waypt_list('4')
    bot_1.add_waypoint_from_waypt_list('u1')
    bot_1.add_waypoint_from_waypt_list('4')
    bot_1.add_waypoint_from_waypt_list('u1')
    bot_1.add_waypoint_from_waypt_list('4')
    bot_1.add_waypoint_from_waypt_list('u1')    

    while not rospy.is_shutdown():


        rate.sleep()

    print("Finished!")

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass