#!/usr/bin/env python
#encoding=utf-8

import sys
sys.path.append("/home/ubuntu484/catkin_ws/src/amigobot_LTL/src/LOMAP_Custom/")               # root path: amigobot_LTL/src

import rospy
import bot_libs.bot as bot
import bot_libs.bot_ts as bot_ts

# uncomment the correspongding case to represent run by amigobot
global time_to_wait
time_to_wait = 10        # seconds

# CASE 2
prefixes = [['u1', 'u1', 'u1', '4', 'u1'],
            ['u2', '10', '11', '23'],
            ['2', '21', '12', '1'],
            ['6', '7', '8', '25']]
suffix_cycles = [['4', '5', '5', '27', '28', '28', 'g4', '28', '21', '22', '23', '9', '10', 'u2', '10', '11', '11', '23', '24', '24', 'g2', '24', '25', '26', '27', '3', '4', 'u1'],
                 ['24', '25', '26', '26', '26', 'g3', '26', '27', '3', '4', 'u1', '4', '5', '27', '28', '21', '22', '22', '22', 'g1', '22', '23', '9', '10', 'u2', '10', '11', '23'],
                 ['2', '21', '12', '1', '2', '21', '12', '1', '2', '21', '12', '1', '2', '21', '12', '1', '2', '21', '12', '1', '2', '21', '12', '1', '2', '21', '12', '1'],
                 ['6', '7', '8', '25', '6', '7', '8', '25', '6', '7', '8', '25', '6', '7', '8', '25', '6', '7', '8', '25', '6', '7', '8', '25', '6', '7', '8', '25']]


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

    bot_1 = bot_ts.turtlebot_TS(name='amigobot_1', yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr2013_real_amigobot/robot_1.yaml',
                                                   map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr2013_real_amigobot/map.yaml',
                                                   time_to_wait = time_to_wait)
    bot_2 = bot_ts.turtlebot_TS(name='amigobot_2', yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr2013_real_amigobot/robot_2.yaml',
                                                   map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr2013_real_amigobot/map.yaml',
                                                   time_to_wait = time_to_wait)
    bot_3 = bot_ts.turtlebot_TS(name='amigobot_3', yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr2013_real_amigobot/robot_3.yaml',
                                                   map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr2013_real_amigobot/map.yaml',
                                                   time_to_wait = time_to_wait)
    bot_4 = bot_ts.turtlebot_TS(name='amigobot_4', yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr2013_real_amigobot/robot_4.yaml',
                                                   map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr2013_real_amigobot/map.yaml',
                                                   time_to_wait = time_to_wait)

    # add prefix
    for i in range(0, prefixes[0].__len__()):
        bot_1.add_waypoint_from_waypt_list(prefixes[0][i])
    for i in range(0, prefixes[1].__len__()):
        bot_2.add_waypoint_from_waypt_list(prefixes[1][i])
    for i in range(0, prefixes[2].__len__()):
        bot_3.add_waypoint_from_waypt_list(prefixes[2][i])
    for i in range(0, prefixes[3].__len__()):
        bot_4.add_waypoint_from_waypt_list(prefixes[3][i])

    # add suffix
    for i in range(0, suffix_cycles[0].__len__()):
        bot_1.add_waypoint_from_waypt_list(suffix_cycles[0][i])
    for i in range(0, suffix_cycles[1].__len__()):
        bot_2.add_waypoint_from_waypt_list(suffix_cycles[1][i])
    for i in range(0, suffix_cycles[2].__len__()):                      # range(0, ...)
        bot_3.add_waypoint_from_waypt_list(suffix_cycles[2][i])
    for i in range(0, suffix_cycles[3].__len__()):
        bot_4.add_waypoint_from_waypt_list(suffix_cycles[3][i])

    # print total cost
    print('[total cost]' + bot_1.name + 'total cost: ' + str(calculate_final_time(bot_1)))
    print('[total cost]' + bot_2.name + 'total cost: ' + str(calculate_final_time(bot_2)))
    print('[total cost]' + bot_3.name + 'total cost: ' + str(calculate_final_time(bot_3)))
    print('[total cost]' + bot_4.name + 'total cost: ' + str(calculate_final_time(bot_4)))

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

        if bot_4.is_all_done == True:
            for i in range(0, suffix_cycles[3].__len__()):
                bot_4.add_waypoint_from_waypt_list(suffix_cycles[3][i])
        '''

        rate.sleep()

    print("Finished!")

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass