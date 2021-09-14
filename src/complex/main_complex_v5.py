#!/usr/bin/env python
#encoding=utf-8

import sys
sys.path.append("/home/ubuntu484/catkin_ws/src/amigobot_LTL/src/LOMAP_Custom/")               # root path: amigobot_LTL/src
sys.path.append("/home/ubuntu484/catkin_ws/src/amigobot_LTL/src/") 

import rospy
import bot_libs.bot as bot
import bot_libs.bot_ts as bot_ts

# uncomment the correspongding case to represent run by amigobot
global time_to_wait, v_max
time_to_wait = 10        # seconds
v_max_1_2 = 0.225        # m/s
v_max_3   = 0.225        # 0.245 m/s if craching        default: 0.205 m/s


def calculate_final_time(bot_w_ts):
    total_time = 0
    for index in bot_w_ts.waypt:
        if index[3] != None:
            total_time += index[3]  # [x, y, yaw, maximum_time]
    return total_time

def main():
    rospy.init_node('ijrr2013_ca_improv', anonymous=False)

    rate = rospy.Rate(50)	# Hz
    rospy.sleep(10)

    # CASE 2 LARGER
    prefixes = [['u1', '22', '23', '22', 'u1', 'u2', '11', '12'],
                ['u2', 'u1', 'u1', 'u2', '16', 'u2', 'u1', '22'],
                ['11', '13', '12', '11', '13', '12', '11', '13'],
                ['26', '25', '21', '26', '25', '21', '26'],
                ['23', '24', '25', '21', 'u1', '22', '23', '24']]

    suffix_cycles = [['14', '15', 'g2', '15', '16', 'u2', '11', '12', '14', '15', 'g2', '15', '16', 'u2', '11', '12'],
                     ['23', '24', 'g3', '24', '25', '21', 'u1', '22', '22', '23', '24', 'g3', '24', '25', '21', 'u1', '22', '22'],
                     ['12', '11', '13', '12', '11', '13', '12', '11', '13', '12', '11', '13', '12', '11', '13', '12', '11', '13'],
                     ['25', '21', '26', '25', '21', '26', '25', '21', '26', '25', '21', '26', '25', '21', '26'],
                     ['25', '21', 'u1', '22', '23', '24', '25', '21', 'u1', '22', '23', '24', '25', '21', 'u1', '22', '23', '24']]

    
    bot_1 = bot_ts.turtlebot_TS(name='amigobot_1', yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/complex/robot_1.yaml',      
                                                   map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/complex/map_controllable.yaml',
                                                   time_to_wait = time_to_wait, u_dist_max = v_max_1_2)
    bot_2 = bot_ts.turtlebot_TS(name='amigobot_2', yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/complex/robot_2.yaml',
                                                   map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/complex/map_controllable.yaml',
                                                   time_to_wait = time_to_wait, u_dist_max = v_max_1_2)
    bot_3 = bot_ts.turtlebot_TS(name='amigobot_3', yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/complex/robot_3.yaml', 
                                                   map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/complex/map_uncontrollable.yaml',
                                                   time_to_wait = time_to_wait, u_dist_max = v_max_3)
    bot_4 = bot_ts.turtlebot_TS(name='amigobot_4', yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/complex/robot_4.yaml',  
                                                   map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/complex/map_uncontrollable.yaml',
                                                   time_to_wait = time_to_wait, u_dist_max = v_max_3)
    bot_5 = bot_ts.turtlebot_TS(name='amigobot_5', yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/complex/robot_5.yaml', 
                                                   map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/complex/map_uncontrollable.yaml',
                                                   time_to_wait = time_to_wait, u_dist_max = v_max_3)

    # add prefix
    for i in range(0, prefixes[0].__len__()):
        bot_1.add_waypoint_from_waypt_list(prefixes[0][i])
    for i in range(0, prefixes[1].__len__()):
        bot_2.add_waypoint_from_waypt_list(prefixes[1][i])
    for i in range(0, prefixes[2].__len__()):
        bot_3.add_waypoint_from_waypt_list(prefixes[2][i])
    for i in range(0, prefixes[3].__len__()):
        bot_4.add_waypoint_from_waypt_list(prefixes[3][i])
    for i in range(0, prefixes[4].__len__()):
        bot_5.add_waypoint_from_waypt_list(prefixes[4][i])

    # add suffix
    for i in range(0, suffix_cycles[0].__len__()):
        bot_1.add_waypoint_from_waypt_list(suffix_cycles[0][i])
    for i in range(0, suffix_cycles[1].__len__()):
        bot_2.add_waypoint_from_waypt_list(suffix_cycles[1][i])
    for i in range(0, suffix_cycles[2].__len__()):                      # range(0, ...)
        bot_3.add_waypoint_from_waypt_list(suffix_cycles[2][i])
    for i in range(0, suffix_cycles[3].__len__()):                      # range(0, ...)
        bot_4.add_waypoint_from_waypt_list(suffix_cycles[3][i])
    for i in range(0, suffix_cycles[4].__len__()):                      # range(0, ...)
        bot_5.add_waypoint_from_waypt_list(suffix_cycles[4][i])

    # print total cost
    print('[total cost]' + bot_1.name + ' total cost: ' + str(calculate_final_time(bot_1)))
    print('[total cost]' + bot_2.name + ' total cost: ' + str(calculate_final_time(bot_2)))
    print('[total cost]' + bot_3.name + ' total cost: ' + str(calculate_final_time(bot_3)))   
    print('[total cost]' + bot_4.name + ' total cost: ' + str(calculate_final_time(bot_4)))   
    print('[total cost]' + bot_5.name + ' total cost: ' + str(calculate_final_time(bot_5)))   



    while not rospy.is_shutdown():
        # add suffix-cycles

        rate.sleep()

    print("Finished!")

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass
