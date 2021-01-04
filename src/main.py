#!/usr/bin/env python
#encoding=utf-8

import sys
import rospy
import math
from geometry_msgs.msg import Twist
from amigobot_lib.bot import amigobot, amigobot_xyControl, amigobot_w_move_base
from amigobot_lib.bot_ts import amigobot_TS

import sys
sys.path.append(sys.path[0] + "/LOMAP-Custom/")               # root path: amigobot_LTL/src
import lomap
import lomap.algorithms.multi_agent_optimal_run_ca as ca
import view

def main():
    try:
        amigobot_w_move_base()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass
