#!/usr/bin/env python
#encoding=utf-8

import sys
sys.path.append("/home/ghost/catkin_ws_ros/src/amigobot_LTL/src/LOMAP_Custom/")               # root path: amigobot_LTL/src

import rospy
import bot_libs.bot as bot
import bot_libs.bot_ts as bot_ts

global time_to_wait
time_to_wait = 5        # seconds

def main():
    rospy.init_node('ijrr2013_ca_improv', anonymous=False)

    rate = rospy.Rate(120)	# 50Hz

    rospy.sleep(8)
                                                       
    bot_1 = bot.turtlebot(x=0, y=6.8, name='amigobot_1')


    bot_1.add_motion('left_turn')
    bot_1.add_motion('forward', 3)
    bot_1.add_motion('right_turn')
    bot_1.add_motion('forward', 3)
    bot_1.add_motion('wait', 2)
    bot_1.add_motion('u_turn')
    bot_1.add_motion('forward', 5)
    bot_1.add_motion('u_turn')

    while not rospy.is_shutdown():
        rate.sleep()

    print("Finished!")

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass