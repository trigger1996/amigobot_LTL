#!/usr/bin/env python
#encoding=utf-8

import sys
sys.path.append("/home/ubuntu484/catkin_ws/src/amigobot_LTL/src/LOMAP_Custom/")               # root path: amigobot_LTL/src
                                                                                              # /home/ghost/catkin_ws_ros/src/amigobot_LTL/src/LOMAP_Custom/
import rospy
import bot_libs.bot as bot
import bot_libs.bot_ts as bot_ts

global time_to_wait
time_to_wait = 10        # seconds

def main():
    rospy.init_node('ijrr2013_ca_improv', anonymous=False)

    rate = rospy.Rate(50)	# 50Hz

    rospy.sleep(8)

    bot_1 = bot_ts.turtlebot_TS(name='amigobot_1', model=None, yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr_2013_improv/robot_1.yaml',      # /home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/
                                                               map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr_2013_improv/map.yaml',
                                                               time_to_wait = time_to_wait)
    bot_2 = bot_ts.turtlebot_TS(name='amigobot_2', model=None, yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr_2013_improv/robot_2.yaml',
                                                               map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr_2013_improv/map.yaml',
                                                               time_to_wait = time_to_wait)
    bot_3 = bot_ts.turtlebot_TS(name='amigobot_3', model=None, yaml_file='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr_2013_improv/robot_3.yaml',
                                                               map_file ='/home/ubuntu484/catkin_ws/src/amigobot_LTL/model/ijrr_2013_improv/map.yaml',
                                                               time_to_wait = time_to_wait)

    is_modifible = [True, True, False]
    prefix_length, prefixes, suffix_cycle_cost, suffix_cycles, team_prefix, team_suffix_cycle = \
        ca.multi_agent_optimal_run_ca(ts_tuple, formula, opt_prop, is_modifible, is_pp=False)

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