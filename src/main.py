#!/usr/bin/env python
#encoding=utf-8

import sys
sys.path.append("/home/ubuntu484/catkin_ws/src/amigobot_LTL/src/LOMAP_Custom/")               # root path: amigobot_LTL/src
                                                                                              # /home/ghost/catkin_ws_ros/src/amigobot_LTL/src/LOMAP_Custom/
import rospy
import bot_libs.bot as bot
import bot_libs.bot_ts as bot_ts
import lomap
from lomap import Ts, Timer
import logging
from collections import namedtuple

# custom packages
import view
import lomap.algorithms.multi_agent_optimal_run_ca as ca

# Logger configuration
logger = logging.getLogger(__name__)

global time_to_wait, v_max
time_to_wait = 10        # seconds
v_max_1_2 = 0.195        # m/s
v_max_3   = 0.205        # 0.245 m/s if craching        default: 0.205 m/s

def main():
    rospy.init_node('ijrr2013_ca_improv', anonymous=False)

    rate = rospy.Rate(50)	# 50Hz

    rospy.sleep(8)

    bot_1 = bot_ts.turtlebot_TS(name='amigobot_1', yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr_2013_improv/robot_1.yaml',      # /home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/
                                                   map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr_2013_improv/map.yaml',
                                                   time_to_wait = time_to_wait, u_dist_max = v_max_1_2)
    bot_2 = bot_ts.turtlebot_TS(name='amigobot_2', yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr_2013_improv/robot_2.yaml',
                                                   map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr_2013_improv/map.yaml',
                                                   time_to_wait = time_to_wait, u_dist_max = v_max_1_2)
    bot_3 = bot_ts.turtlebot_TS(name='amigobot_3', yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr_2013_improv/robot_3_inv.yaml',   # robot_3_inv_larger.yaml   robot_3_inv.yaml
                                                   map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr_2013_improv/map.yaml',
                                                   time_to_wait = time_to_wait, u_dist_max = v_max_3)

    ts_tuple = (bot_1, bot_2, bot_3)

    # CASE 2
    #formula = ('[]<>gather && [](gather->(r1gather && r2gather)) '
    #           '&& [](r1gather -> X(!r1gather U r1upload)) '
    #           '&& [](r2gather -> X(!r2gather U r2upload))')
    #opt_prop = set(['r1gather','r2gather'])

    # CASE 3
    #formula = ('[]<>gather && [](gather->(r1gather && r2gather)) '
    #           '&& [](r1gather -> X(!r1gather U r1upload)) '
    #           '&& [](r2gather -> X(!r2gather U r2upload)) '
    #           '&& [](!(r1gather1 && r2gather1) && !(r1gather2 && r2gather2)'
    #           '&& !(r1gather3 && r2gather3) && !(r1gather4 && r2gather4))')
    #opt_prop = set(['r1gather','r2gather'])

    # CASE 4
    formula = ('[]<>gather && [](gather->(r1gather4 && r2gather2)) '
               '&& [](r1gather -> X(!r1gather U r1upload)) '
               '&& [](r2gather -> X(!r2gather U r2upload))')
    opt_prop = set(['r1gather4','r2gather2'])


    is_modifible = [True, True, False]
    prefix_length, prefixes, suffix_cycle_cost, suffix_cycles, team_prefix, team_suffix_cycle = \
        ca.multi_agent_optimal_run_ca(ts_tuple, formula, opt_prop, is_modifible, is_pp=True)

    logger.info('Cost: %d', suffix_cycle_cost)
    logger.info('Prefix length: %d', prefix_length)
    # Find the controls that will produce this run
    control_prefixes = []
    control_suffix_cycles = []
    for i in range(0, len(ts_tuple)):
        ts = ts_tuple[i]
        control_prefixes.append(ts.controls_from_run(prefixes[i]))
        control_suffix_cycles.append(ts.controls_from_run(suffix_cycles[i]))
        logger.info('%s run prefix: %s', ts.name, prefixes[i])
        logger.info('%s control perfix: %s', ts.name, control_prefixes[i])
        logger.info('%s suffix cycle: %s', ts.name, suffix_cycles[i])
        logger.info('%s control suffix cycle: %s', ts.name,
                                                   control_suffix_cycles[i])    

        if suffix_cycles[i][0] == prefix[i][prefix.__len__() - 1]:
            suffix_cycles[i].pop(0)

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

    while not rospy.is_shutdown():
        rate.sleep()

    print("Finished!")

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass