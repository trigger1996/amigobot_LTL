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
    def __init__(self, name='amigobot_1', model=None, x = None, y = None, yaw = None, yaml_file='robot.yaml', map_file='map.yaml'):
        super(Ts, self).__init__()
        super(turtlebot_TS, self).__init__(name)

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
        self.yaw_init_tab  = dict()
        self.load_from_map(map_file)

        # override waypoint varibles
        if x == None or y == None:
            self.x = self.waypoint_dict[list(self.init)[0]][0]
            self.y = self.waypoint_dict[list(self.init)[0]][1]
            self.target_x = self.x      # current target
            self.target_y = self.y
            self.target_yaw = self.yaw
            self.target_x_last = None
            self.target_y_last = None
            self.target_yaw_last = None
            self.waypt = []             # next target list [[x, y, yaw]]

    def load_from_map(self, map_file):
        f = io.open(map_file, 'r', encoding='utf8')
        data = load(f, Loader=Loader)

        self.waypoint_dict = data['waypoint']
        self.yaw_init_tab  = data['initial_yaw']

    def add_waypoint_from_waypt_list(self, waypt_name):
        x = self.waypoint_dict[waypt_name][0]
        y = self.waypoint_dict[waypt_name][1]

        print('[Command]: ' + self.name + ": " + str(waypt_name) + "  (" + str(x) + ", " + str(y) + ")")
        self.add_waypoint(x, y)