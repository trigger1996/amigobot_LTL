#!/usr/bin/env python
#encoding=utf-8

import sys
sys.path.append("/home/ubuntu484/catkin_ws/src/amigobot_LTL/src/LOMAP_Custom/")               # root path: amigobot_LTL/src
sys.path.append("/home/ubuntu484/catkin_ws/src/amigobot_LTL/src/") 

import rospy
from bot_libs.amigobot_ts import amigobot_TS

# uncomment the correspongding case to represent run by amigobot
global time_to_wait, v_max
time_to_wait = 10        # seconds
v_max_1_2 = 0.155        # m/s
v_max_3   = 0.155        # 0.245 m/s if craching        default: 0.095 m/s


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

    # CASE 4 INV LARGER
    # Check .yaml before launching
    '''
    prefixes = [['u1', 'u1'],
                ['u2', 'u2'],
                ['23']]
    suffix_cycles = [['27', '28', 'g4', '28', '27', 'u1', '27', '28', '28', '28', '28', '28', 'g4', '28', '27', 'u1', '27', '28', 'g4', '28', '27', 'u1', 'u1', 'u1', 'u1', 'u1', '27', '28', 'g4', '28', '27', 'u1'],
                     ['23', '24', 'g2', '24', '23', 'u2', 'u2', 'u2', 'u2', 'u2', '23', '24', 'g2', '24', '23', 'u2', '23', '24', 'g2', '24', '23', 'u2', '23', '24', '24', '24', '24', '24', 'g2', '24', '23', 'u2'],
                     ['22', '21', '28', '27', '26', '25', '24', '23', '22', '21', '28', '27', '26', '25', '24', '23', '22', '21', '28', '27', '26', '25', '24', '23']]
    '''
    prefixes = [['u1', 'u1']]
    suffix_cycles = [['27', '28', 'g4', '28', '21', '22', 'g1', '22', '23', 'u2', 'u2', '23', '24', 'g2', '24', '25', '26', 'g3', '26', '27', '28', 'g4', '28', '27', 'u1']]
    bot_1 = amigobot_TS(name='amigobot_1', yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/physical/3_vehicles/robot_1.yaml',      # /home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/
                                                   map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/physical/3_vehicles/map.yaml',
                                                   time_to_wait = time_to_wait, u_dist_max = v_max_1_2)

    # robot_3_inv_larger.yaml
    # robot_3_inv.yaml

    # add prefix
    for i in range(0, prefixes[0].__len__()):
        bot_1.add_waypoint_from_waypt_list(prefixes[0][i])


    # add suffix
    for i in range(0, suffix_cycles[0].__len__()):
        bot_1.add_waypoint_from_waypt_list(suffix_cycles[0][i])


    # print total cost
    print('[total cost]' + bot_1.name + ' total cost: ' + str(calculate_final_time(bot_1)))

    while not rospy.is_shutdown():
        # add suffix-cycles

        rate.sleep()

    print("Finished!")

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass
