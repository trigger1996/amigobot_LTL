#!/usr/bin/env python
#encoding=utf-8

import sys
sys.path.append("/home/ubuntu484/catkin_ws/src/amigobot_LTL/LOMAP_Custom/")               # root path: amigobot_LTL/src

import rospy
import bot_libs.bot as bot
import bot_libs.bot_ts as bot_ts

global time_to_wait
time_to_wait = 10        # seconds

def main():
    rospy.init_node('ijrr2013_ca_improv', anonymous=False)

    rate = rospy.Rate(50)	# 50Hz

    rospy.sleep(8)

    bot_1 = bot_ts.turtlebot_TS(name='amigobot_1', model=None, yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr_2013_improv/robot_1.yaml',
                                                               map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr_2013_improv/map.yaml',
                                                               time_to_wait = time_to_wait)
    bot_2 = bot_ts.turtlebot_TS(name='amigobot_2', model=None, yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr_2013_improv/robot_2.yaml',
                                                               map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr_2013_improv/map.yaml',
                                                               time_to_wait = time_to_wait)
    bot_3 = bot_ts.turtlebot_TS(name='amigobot_3', model=None, yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr_2013_improv/robot_3.yaml',
                                                               map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr_2013_improv/map.yaml',
                                                               time_to_wait = time_to_wait)

    
    bot_1.add_waypoint_from_waypt_list('4')
    bot_1.add_waypoint_from_waypt_list('4')
    bot_1.add_waypoint_from_waypt_list('5')
    bot_1.add_waypoint_from_waypt_list('6')
    bot_1.add_waypoint_from_waypt_list('7')
    bot_1.add_waypoint_from_waypt_list('8')
    bot_1.add_waypoint_from_waypt_list('9')
    bot_1.add_waypoint_from_waypt_list('10')
    bot_1.add_waypoint_from_waypt_list('u2')
    bot_1.add_waypoint_from_waypt_list('10')   
    bot_1.add_waypoint_from_waypt_list('11')
    bot_1.add_waypoint_from_waypt_list('12')
    bot_1.add_waypoint_from_waypt_list('1')
    bot_1.add_waypoint_from_waypt_list('2')
    bot_1.add_waypoint_from_waypt_list('3')
    bot_1.add_waypoint_from_waypt_list('4')
    bot_1.add_waypoint_from_waypt_list('u1')    
    bot_1.add_waypoint_from_waypt_list('u1')    # for showing arriving time
    
    '''
    bot_1.add_waypoint_from_waypt_list('4')
    bot_1.add_waypoint_from_waypt_list('4')
    bot_1.add_waypoint_from_waypt_list('5')
    bot_1.add_waypoint_from_waypt_list('27')
    bot_1.add_waypoint_from_waypt_list('28')
    bot_1.add_waypoint_from_waypt_list('g4')
    bot_1.add_waypoint_from_waypt_list('28')
    bot_1.add_waypoint_from_waypt_list('21')
    bot_1.add_waypoint_from_waypt_list('22')
    bot_1.add_waypoint_from_waypt_list('g1')
    bot_1.add_waypoint_from_waypt_list('22')
    bot_1.add_waypoint_from_waypt_list('23')
    bot_1.add_waypoint_from_waypt_list('24')
    bot_1.add_waypoint_from_waypt_list('g2')
    bot_1.add_waypoint_from_waypt_list('24')
    bot_1.add_waypoint_from_waypt_list('25')
    bot_1.add_waypoint_from_waypt_list('26')
    bot_1.add_waypoint_from_waypt_list('g3')
    bot_1.add_waypoint_from_waypt_list('26')
    bot_1.add_waypoint_from_waypt_list('27')
    bot_1.add_waypoint_from_waypt_list('3')    
    bot_1.add_waypoint_from_waypt_list('4')
    bot_1.add_waypoint_from_waypt_list('u1')    
    bot_1.add_waypoint_from_waypt_list('u1')    # for showing arriving time
    '''

    while not rospy.is_shutdown():
        rate.sleep()

    print("Finished!")

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass