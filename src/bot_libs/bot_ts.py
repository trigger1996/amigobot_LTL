#!/usr/bin/env python
#encoding=utf-8

import rospy
import io
from yaml import load, dump #, safe_load as load
try: # try using the libyaml if installed
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError: # else use default PyYAML loader and dumper
    from yaml import Loader, Dumper

import math
from bot import turtlebot
from lomap import Model
from lomap import Ts

class turtlebot_TS(turtlebot, Ts):
    def __init__(self, name='amigobot_1', model=None, x = None, y = None, yaw = None, time_to_wait = 1, yaml_file='robot.yaml', map_file='map.yaml'):
        super(Ts, self).__init__()
        super(turtlebot_TS, self).__init__(name, time_to_wait=time_to_wait)

        # import data from TS
        ts_raw = self.load(yaml_file)

        self.current  = ts_raw.current
        self.directed = ts_raw.directed
        self.final    = ts_raw.final
        self.g        = ts_raw.g
        self.init     = ts_raw.init
        self.multi    = ts_raw.multi

        # import data from map
        self.waypoint_dict = dict()
        self.final_yaw_tab = dict()
        self.load_from_map(map_file)

        #
        self.last_target_waypt = None
        self.next_target_waypt = list(self.init)[0]

        # override waypoint varibles
        if x == None or y == None:
            self.x = self.waypoint_dict[list(self.init)[0]][0]
            self.y = self.waypoint_dict[list(self.init)[0]][1]
            self.target_x = self.x      # current target
            self.target_y = self.y
            self.target_yaw = None
            self.target_x_last = None
            self.target_y_last = None
            self.target_yaw_last = None
            self.waypt = []             # next target list [[x, y, yaw]]

    def load_from_map(self, map_file):
        f = io.open(map_file, 'r', encoding='utf8')
        data = load(f, Loader=Loader)

        self.waypoint_dict = data['waypoint']
        self.final_yaw_tab  = data['initial_yaw']

    def add_waypoint_from_waypt_list(self, waypt_name):
        self.last_target_waypt = self.next_target_waypt
        self.next_target_waypt = waypt_name

        x = self.waypoint_dict[waypt_name][0]
        y = self.waypoint_dict[waypt_name][1]
        yaw = self.find_motion_target_yaw(self.last_target_waypt, self.next_target_waypt)

        print('[Command]: ' + self.name + ": " + str(waypt_name) + "  (" + str(x) + ", " + str(y) + ", " + str(yaw) + ")")
        self.add_waypoint(x, y, yaw)

    def find_motion_target_yaw(self, src, dst):

        for index in self.final_yaw_tab:
            # find the corresponding final yaw for each motion
            if src == index[0]:
                if dst == index[1]:
                    return index[2]['yaw']

        # must ensure ALL possibilities do not exist, so there should 2 for-loop
        for index in self.final_yaw_tab:
            # else if the motion is go-back, vehicle should make the same turn
            # wired but useful, if route is 2 --> 1 --> 12, when arrving 1, the yaw is 90 not -180, depending on 2 rather than 1, but this rule is easy and useful, if depend on 1, it will harder to determind how to reach 1 (if route is 5 --> 4 --> 3, 4 can be arrived from both u1 and 5)
            if src == index[1]:
                if dst == index[0]:
                    return index[2]['yaw']      
                    
        # if motion does not found, do not restrict the final yaw
        return None
