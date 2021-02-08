#!/usr/bin/env python
#encoding=utf-8

import sys
sys.path.append("/home/ghost/catkin_ws_ros/src/amigobot_LTL/src/LOMAP_Custom/")               # root path: amigobot_LTL/src

import rospy
import bot_libs.bot as bot
import bot_libs.bot_ts as bot_ts
import bot_libs.bot_w_time as bot_w_time

global time_to_wait
time_to_wait = 5        # seconds

def main():
    rospy.init_node('ijrr2013_ca_improv', anonymous=False)

    rate = rospy.Rate(50)	# 50Hz

    rospy.sleep(8)

    bot_1 = bot_w_time.turtlebot(x=0, y=6.8, name='amigobot_1', time_to_wait=time_to_wait)                                                           

    bot_1.add_waypoint( 0,  0)    
    bot_1.add_waypoint(-3,  0)
    bot_1.add_waypoint(-3, -3)
    bot_1.add_waypoint(-3, -3)    
    bot_1.add_waypoint( 0, -3)
    bot_1.add_waypoint( 0,  0)

    while not rospy.is_shutdown():
        rate.sleep()

    print("Finished!")

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass