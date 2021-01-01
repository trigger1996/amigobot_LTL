#encoding=utf-8

import sys
sys.path.append(sys.path[0] + "/LOMAP-Custom/")               # root path: amigobot_LTL/src

import io
from yaml import load, dump #, safe_load as load
try: # try using the libyaml if installed
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError: # else use default PyYAML loader and dumper
    from yaml import Loader, Dumper


from bot import amigobot_xyControl
from lomap import Model
from lomap import Ts

class amigobot_TS(amigobot_xyControl, Ts):
    def __init__(self, name='RosAria', yaml_file='robot.yaml', map_file='map.yaml'):
        super(Ts, self).__init__()
        super(amigobot_TS, self).__init__(name)

        # import data from TS
        ts_raw = self.load(yaml_file)

        self.current  = ts_raw.current
        self.directed = ts_raw.directed
        self.final    = ts_raw.final
        self.g        = ts_raw.g
        self.init     = ts_raw.init
        self.multi    = ts_raw.multi

        # import data from map
        self.waypoint = dict()
        self.load_from_map(map_file)

        self.x_initial = self.waypoint[list(self.init)[0]][0]
        self.y_initial = self.waypoint[list(self.init)[0]][1]

    def load_from_map(self, map_file):
        f = io.open(map_file, 'r', encoding='utf8')
        data = load(f, Loader=Loader)

        self.waypoint = data['waypoint']

    def add_waypoint_from_waypt_list(self, waypt_name):
        # FL->RF
        # x = y
        # y = -x
        x =   self.waypoint[waypt_name][1] - self.y_initial
        y = -(self.waypoint[waypt_name][0] - self.x_initial)
        print('[Command]: ' + self.name + ": (" + str(x) + ", " + str(y) + ")")
        self.add_waypoint(x, y)
