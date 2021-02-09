#!/usr/bin/env python
#encoding=utf-8

import sys
sys.path.append("/home/ghost/catkin_ws_ros/src/amigobot_LTL/src/LOMAP_Custom/")               # root path: amigobot_LTL/src

import rospy
import bot_libs.bot as bot
import bot_libs.bot_ts as bot_ts

# uncomment the correspongding case to represent run by amigobot
global time_to_wait
time_to_wait = 10        # seconds

# CASE 2
prefixes = [['u1', '4', '5', '27', '28', '28', '21', '21', '12'],
            ['u2', 'u2', 'u2', '10', 'u2', '10', '10', '10', '10', 'u2', 'u2', 'u2'],
            ['11', '12', '1', '2', '21', '22', '23']]
suffix_cycles = [['1', '2', '21', '22', 'g1', '22', '23', '9', '10', 'u2', '10', '11', '23', '24', '25', '26', 'g3', '26', '27', '3', '4', 'u1', '4', '5', '6', '7', '8', '25', '26', 'g3', '26', '27', '3', '4', 'u1', '4', '5', '27', '28', '28', '21', '22', '22', 'g1', '22', '23', '9', '10', 'u2', '10', '11', '12'],
                 ['10', '11', '23', '24', '24', 'g2', '24', '25', '26', '27', '3', '4', 'u1', '4', '5', '27', '28', 'g4', '28', '28', '21', '22', '22', '23', '9', '10', 'u2', '10', '11', '23', '24', 'g2', '24', '24', '25', '26', '27', '3', '4', 'u1', '4', '5', '27', '28', '28', 'g4', '28', '21', '22', '23', '23', '9', '10', 'u2'],
                 ['9', '10', '11', '12', '1', '2', '21', '22', '23', '9', '10', '11', '12', '1', '2', '21', '22', '23', '9', '10', '11', '12', '1', '2', '21', '22', '23', '9', '10', '11', '12', '1', '2', '21', '22', '23', '9', '10', '11', '12', '1', '2', '21', '22', '23']]

'''
# CASE 3
'''
prefixes = [['u1', '4', '5', '27', '28', '28', '21', '21', '12'],
            ['u2', 'u2', 'u2', '10', 'u2', '10', '10', '10', '10', 'u2', 'u2', 'u2'],
            ['11', '12', '1', '2', '21', '22', '23']]
suffix_cycles = [['1', '2', '21', '22', 'g1', '22', '23', '9', '10', 'u2', '10', '11', '23', '24', '25', '26', 'g3', '26', '27', '3', '4', 'u1', '4', '5', '6', '7', '8', '25', '26', 'g3', '26', '27', '3', '4', 'u1', '4', '5', '27', '28', '28', '21', '22', '22', 'g1', '22', '23', '9', '10', 'u2', '10', '11', '12'],
                 ['10', '11', '23', '24', '24', 'g2', '24', '25', '26', '27', '3', '4', 'u1', '4', '5', '27', '28', 'g4', '28', '28', '21', '22', '22', '23', '9', '10', 'u2', '10', '11', '23', '24', 'g2', '24', '24', '25', '26', '27', '3', '4', 'u1', '4', '5', '27', '28', '28', 'g4', '28', '21', '22', '23', '23', '9', '10', 'u2'],
                 ['9', '10', '11', '12', '1', '2', '21', '22', '23', '9', '10', '11', '12', '1', '2', '21', '22', '23', '9', '10', '11', '12', '1', '2', '21', '22', '23', '9', '10', '11', '12', '1', '2', '21', '22', '23', '9', '10', '11', '12', '1', '2', '21', '22', '23']]

'''
# CASE 4
'''
prefixes = [['u1', '4', '5', '27', '3', '3', '4', 'u1'],
            ['u2', '10', 'u2', '10', 'u2', 'u2', 'u2', 'u2', 'u2'],
            ['11', '12', '1', '2', '21', '22']]
suffix_cycles = [['4', '5', '27', '28', 'g4', '28', '21', '12', '1', '2', '3', '4', 'u1', '4', '5', '27', '28', '28', 'g4', '28', '28', '21', '12', '1', '2', '2', '3', '4', 'u1', '4', '5', '27', '28', 'g4', '28', '21', '12', '1', '2', '3', '3', '4', 'u1'],
                 ['10', '11', '23', '23', '24', 'g2', '24', '25', '6', '7', '8', '9', '10', 'u2', 'u2', '10', '11', '23', '24', 'g2', '24', '25', '6', '7', '8', '8', '8', '9', '10', 'u2', '10', '11', '23', '24', 'g2', '24', '25', '6', '7', '8', '9', '10', 'u2'],
                 ['23', '9', '10', '11', '12', '1', '2', '21', '22', '23', '9', '10', '11', '12', '1', '2', '21', '22', '23', '9', '10', '11', '12', '1', '2', '21', '22', '23', '9', '10', '11', '12', '1', '2', '21', '22']]


def calculate_final_time(bot_w_ts):
    total_time = 0
    for index in bot_w_ts.waypt:
        if index[3] != None:
            total_time += index[3]  # [x, y, yaw, maximum_time]
    return total_time

def main():
    rospy.init_node('ijrr2013_ca_improv', anonymous=False)

    rate = rospy.Rate(50)	# 50Hz
    rospy.sleep(10)

    bot_1 = bot_ts.turtlebot_TS(name='amigobot_1', yaml_file='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/robot_1.yaml',
                                                   map_file ='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/map.yaml',
                                                   time_to_wait = time_to_wait)
    bot_2 = bot_ts.turtlebot_TS(name='amigobot_2', yaml_file='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/robot_2.yaml',
                                                   map_file ='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/map.yaml',
                                                   time_to_wait = time_to_wait)
    bot_3 = bot_ts.turtlebot_TS(name='amigobot_3', yaml_file='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/robot_3.yaml',
                                                   map_file ='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/map.yaml',
                                                   time_to_wait = time_to_wait)

    # add prefix
    for i in range(0, prefixes[0].__len__()):
        bot_1.add_waypoint_from_waypt_list(prefixes[0][i])
    for i in range(0, prefixes[1].__len__()):
        bot_2.add_waypoint_from_waypt_list(prefixes[1][i])
    for i in range(0, prefixes[2].__len__()):
        bot_3.add_waypoint_from_waypt_list(prefixes[2][i])

    # add suffix
    for i in range(0, suffix_cycles[0].__len__()):
        bot_1.add_waypoint_from_waypt_list(suffix_cycles[0][i])
    for i in range(0, suffix_cycles[1].__len__()):
        bot_2.add_waypoint_from_waypt_list(suffix_cycles[1][i])
    for i in range(0, suffix_cycles[2].__len__()):                      # range(0, ...)
        bot_3.add_waypoint_from_waypt_list(suffix_cycles[2][i])

    # print total cost
    print('[total cost]' + bot_1.name + 'total cost: ' + str(calculate_final_time(bot_1)))
    print('[total cost]' + bot_2.name + 'total cost: ' + str(calculate_final_time(bot_2)))
    print('[total cost]' + bot_3.name + 'total cost: ' + str(calculate_final_time(bot_3)) )   

    while not rospy.is_shutdown():
        # add suffix-cycles
        '''
        if bot_1.is_all_done == True:
            for i in range(0, suffix_cycles[0].__len__()):
                bot_1.add_waypoint_from_waypt_list(suffix_cycles[0][i])

        if bot_2.is_all_done == True:
            for i in range(0, suffix_cycles[1].__len__()):
                bot_2.add_waypoint_from_waypt_list(suffix_cycles[1][i])

        if bot_3.is_all_done == True:
            for i in range(0, suffix_cycles[2].__len__()):
                bot_3.add_waypoint_from_waypt_list(suffix_cycles[2][i])
        '''

        rate.sleep()

    print("Finished!")

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass