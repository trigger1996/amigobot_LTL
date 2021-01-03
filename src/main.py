#!/usr/bin/env python
#encoding=utf-8

import sys
import rospy
import math
from geometry_msgs.msg import Twist
from amigobot_lib.bot import amigobot, amigobot_xyControl
from amigobot_lib.bot_ts import amigobot_TS

import sys
sys.path.append(sys.path[0] + "/LOMAP-Custom/")               # root path: amigobot_LTL/src
import lomap
import lomap.algorithms.multi_agent_optimal_run_ca as ca
import view

def main():
    rospy.init_node('motion_primitive_test', anonymous=False)

    bot_1 = amigobot_TS(name='amigobot_1', yaml_file=sys.path[0] + '/../model/ijrr_2013_improv/robot_1.yaml',
                                           map_file =sys.path[0] + '/../model/ijrr_2013_improv/map.yaml')
    bot_2 = amigobot_TS(name='amigobot_2', yaml_file=sys.path[0] + '/../model/ijrr_2013_improv/robot_2.yaml',
                                           map_file =sys.path[0] + '/../model/ijrr_2013_improv/map.yaml')
    rate = rospy.Rate(25)	# 5Hz

    rospy.sleep(3)
    ts_tuple = (bot_1, bot_2)
    formula = ('[]<>gather && [](gather->(r1gather && r2gather)) '
               '&& [](r1gather -> X(!r1gather U r1upload)) '
               '&& [](r2gather -> X(!r2gather U r2upload))')

    opt_prop = set(['r1gather', 'r2gather'])
    prefix_length, prefixes, suffix_cycle_cost, suffix_cycles, prefix_on_team_ts, suffix_cycle_on_team_ts = \
        ca.multi_agent_optimal_run(ts_tuple,  formula, opt_prop)

    print(prefixes)
    print(suffix_cycles)
    for i in range(0, prefixes[0].__len__()):
        bot_1.add_waypoint_from_waypt_list(prefixes[0][i])
    for i in range(0, prefixes[1].__len__()):
        bot_2.add_waypoint_from_waypt_list(prefixes[1][i])
    for i in range(1, suffix_cycles[0].__len__()):
        bot_1.add_waypoint_from_waypt_list(suffix_cycles[0][i])
    for i in range(0, suffix_cycles[1].__len__()):
        bot_2.add_waypoint_from_waypt_list(suffix_cycles[1][i])

    #view.visualize_animation_w_team_run(ts_tuple, suffix_cycle_on_team_ts)

    while not rospy.is_shutdown():
        if bot_1.is_all_done == True:
            for i in range(1, suffix_cycles[0].__len__()):
                bot_1.add_waypoint_from_waypt_list(suffix_cycles[0][i])

        if bot_2.is_all_done == True:
            for i in range(0, suffix_cycles[1].__len__()):
                bot_2.add_waypoint_from_waypt_list(suffix_cycles[1][i])

        rate.sleep()


    print(233)

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass
