#!/usr/bin/env python
#encoding=utf-8

import sys
sys.path.append("/home/ghost/catkin_ws_ros/src/amigobot_LTL/src/LOMAP_Custom/")               # root path: amigobot_LTL/src
sys.path.append("/home/ghost/catkin_ws_ros/src/amigobot_LTL/src/") 

import rospy
from bot_libs.amigobot_ts import amigobot_TS

# uncomment the correspongding case to represent run by amigobot
global time_to_wait
time_to_wait = 10        # seconds
v_max_1_2 = 0.145        # m/s
v_max_3_4 = 0.145        # 0.245 m/s if craching        default: 0.205 m/s

# CASE 2
prefixes = [['u1', '27'],
            ['u2', '23', '23'],
            ['g3', '26', 'g3', '26'],
            ['22', 'g1', '22', 'g1']]
suffix_cycles = [['28', '21', '21', '22', '23', '23', '24', 'g2', '24', '25', '26', '27', 'u1', '27'],
                 ['23', '24', '25', '26', '27', '28', 'g4', '28', '21', '21', '22', '23', 'u2', '23'],
                 ['g3', '26', 'g3', '26', 'g3', '26', 'g3', '26', 'g3', '26', 'g3', '26', 'g3', '26', 'g3', '26', 'g3', '26', 'g3', '26', 'g3', '26', 'g3', '26'],
                 ['22', 'g1', '22', 'g1', '22', 'g1', '22', 'g1', '22', 'g1', '22', 'g1', '22', 'g1', '22', 'g1', '22', 'g1', '22', 'g1', '22', 'g1', '22', 'g1']]


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

    bot_1 = amigobot_TS(name='amigobot_1', yaml_file='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/physical/4_vehicles/robot_1.yaml',      # /home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/
                                                   map_file ='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/physical/4_vehicles/map.yaml',
                                                   time_to_wait = time_to_wait, u_dist_max = v_max_1_2)
    bot_2 = amigobot_TS(name='amigobot_2', yaml_file='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/physical/4_vehicles/robot_2.yaml',
                                                   map_file ='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/physical/4_vehicles/map.yaml',
                                                   time_to_wait = time_to_wait, u_dist_max = v_max_1_2)
    bot_3 = amigobot_TS(name='amigobot_3', yaml_file='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/physical/4_vehicles/robot_3.yaml',   # robot_3_inv_larger.yaml   robot_3_inv.yaml
                                                   map_file ='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/physical/4_vehicles/map.yaml',
                                                   time_to_wait = time_to_wait, u_dist_max = v_max_3_4)
    bot_4 = amigobot_TS(name='amigobot_4', yaml_file='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/physical/4_vehicles/robot_4.yaml',   # robot_3_inv_larger.yaml   robot_3_inv.yaml
                                                   map_file ='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/physical/4_vehicles/map.yaml',
                                                   time_to_wait = time_to_wait, u_dist_max = v_max_3_4)

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
    #print('[total cost]' + bot_1.name + 'total cost: ' + str(calculate_final_time(bot_1)))
    #print('[total cost]' + bot_2.name + 'total cost: ' + str(calculate_final_time(bot_2)))
    #print('[total cost]' + bot_3.name + 'total cost: ' + str(calculate_final_time(bot_3)))
    #print('[total cost]' + bot_4.name + 'total cost: ' + str(calculate_final_time(bot_4)))

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