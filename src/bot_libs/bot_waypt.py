#!/usr/bin/env python
#encoding=utf-8

import rospy
import io
from yaml import load, dump #, safe_load as load
try: # try using the libyaml if installed
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError: # else use default PyYAML loader and dumper
    from yaml import Loader, Dumper

from math import radians, copysign, sqrt, pow, pi, atan2, fabs
from bot import turtlebot

def normalize_angle(yaw):   # in degree
    while yaw > 180.:
        yaw = yaw - 360.
    while yaw <= -180.:
        yaw = yaw + 360.
    return yaw

class turtlebot_waypt(turtlebot):
    def __init__(self, name='amigobot_1', x = None, y = None, yaw = None, time_to_wait = 1):
        super(turtlebot_waypt, self).__init__(name)

        # waypoint varibles
        self.target_x = self.x      # current target
        self.target_y = self.y
        self.target_yaw = None
        self.target_x_last = None
        self.target_y_last = None
        self.target_yaw_last = None
        self.waypt = []             # next target list [[x, y, yaw]]
        self.is_all_done   = False

        self.x_err_threshold = 0.05
        self.y_err_threshold = 0.05

        self.time_to_wait = time_to_wait


    def motion_control(self):

        if self.is_target_set == False:

            # target arrived
            if self.timestamp_next >= self.time_curr:
                self.timestamp_last = self.timestamp_next

                # assume that each waypoint will fill motion list once, if empty, target waypoint is arrived
                if self.next_motion_list.__len__() == 0:

                    if self.waypt.__len__() == 0:
                        self.is_all_done = True
                    else:
                        self.target_x_last   = self.target_x
                        self.target_y_last   = self.target_y
                        self.target_yaw_last = self.target_yaw

                        [self.target_x, self.target_y, self.target_yaw] = self.waypt.pop(0)

                        # first get current yaw and get rid of errors
                        yaw_in_90 = self.get_current_yaw()
                        x_err = self.target_x - self.target_x_last      # x_err = self.target_x - self.x
                        y_err = self.target_y - self.target_y_last      # y_err = self.target_y - self.y
                        
                        # if waiting
                        if x_err == 0 and y_err == 0:
                            self.add_motion('wait', self.time_to_wait)

                        # if not waiting
                        else:
                            # x first
                            if x_err > self.x_err_threshold:
                                for i in range(0, int(yaw_in_90 / 90.)):
                                    self.add_motion('left_turn')
                            elif x_err < -self.x_err_threshold:
                                for i in range(0, int(-yaw_in_90 / 90.)):
                                    self.add_motion('right_turn')
                            else:
                                pass

                            self.add_motion('forward', fabs(x_err) / self.motion_speed['forward'][0])

                            # y second
                            if y_err > self.y_err_threshold:
                                self.add_motion('left_turn')
                            elif y_err < -self.y_err_threshold:
                                self.add_motion('right_turn')
                            else:
                                pass

                            self.add_motion('forward', fabs(x_err) / self.motion_speed['forward'][0])

                            # turn to target yaw
                            if self.target_yaw == 180. or self.target_yaw == -180.:
                                if y_err > self.y_err_threshold:        # current: 90 degree
                                    self.add_motion('left_turn')
                                elif y_err < -self.y_err_threshold:     # current: -90 degree
                                    self.add_motion('right_turn')
                                else:                                   # current 0
                                    self.add_motion('left_turn')
                                    self.add_motion('left_turn')
                            elif self.target_yaw == 90.:
                                if y_err > self.y_err_threshold:        # current: 90 degree
                                    pass
                                elif y_err < -self.y_err_threshold:     # current: -90 degree
                                    self.add_motion('right_turn')
                                    self.add_motion('right_turn')
                                else:                                   # current 0
                                    self.add_motion('left_turn')                               
                            elif self.target_yaw == 0.:
                                if y_err > self.y_err_threshold:        # current: 90 degree
                                    self.add_motion('right_turn')
                                elif y_err < -self.y_err_threshold:     # current: -90 degree
                                    self.add_motion('left_turn')
                                else:                                   # current 0
                                    pass
                            elif self.target_yaw == -90.:
                                if y_err > self.y_err_threshold:        # current: 90 degree
                                    self.add_motion('right_turn')
                                    self.add_motion('right_turn')                                
                                elif y_err < -self.y_err_threshold:     # current: -90 degree
                                    pass
                                else:                                   # current 0
                                    self.add_motion('right_turn')
                            else:
                                pass

                else:
                    [self.current_motion, self.target_duration] = self.next_motion_list.pop(0)
            
            # next motion
            if self.current_motion == 'forward':
                self.vx = self.motion_speed['forward'][0]
                self.wz = self.motion_speed['forward'][1]            
                self.timestamp_next = self.time_curr + self.target_duration

                self.is_target_set = True
            elif self.current_motion == 'right_turn':
                self.vx = self.motion_speed['right_turn'][0]
                self.wz = self.motion_speed['right_turn'][1]            
                self.timestamp_next = self.time_curr + self.motion_duration['right_turn']

                self.is_target_set = True
            elif self.current_motion == 'left_turn':
                self.vx = self.motion_speed['left_turn'][0]
                self.wz = self.motion_speed['left_turn'][1]            
                self.timestamp_next = self.time_curr + self.motion_duration['left_turn']

                self.is_target_set = True
            elif self.current_motion == 'u_turn':
                self.vx = self.motion_speed['u_turn'][0]
                self.wz = self.motion_speed['u_turn'][1]            
                self.timestamp_next = self.time_curr + self.motion_duration['u_turn']

                self.is_target_set = True
            elif self.current_motion == 'wait':
                self.vx = self.motion_speed['wait'][0]
                self.wz = self.motion_speed['wait'][1]            
                self.timestamp_next = self.time_curr + self.target_duration

                self.is_target_set = True
            else:
                self.vx = self.motion_speed['standby'][0]
                self.wz = self.motion_speed['standby'][1]            
                self.timestamp_next = self.time_curr

                self.is_target_set = False

        if self.timestamp_next > self.time_curr:
            if self.time_curr <= self.timestamp_next - self.motion_cooldown_time:
                self.update_target_vel(self.vx, self.wz)            # normal movement
            else:
                self.update_target_vel(0, 0)                        # let the vehicle slip
        else:
            # if time is up, stop moving
            self.current_motion = 'standby'

            self.vx = self.motion_speed['standby'][0]
            self.wz = self.motion_speed['standby'][1]              

            self.update_target_vel(self.vx, self.wz)
            self.is_target_set = False

    def is_vertex_arrived(self, x, y, threshold = 0.05):
        dist = sqrt((self.x - x) ** 2 + (self.y - y) ** 2)
        if dist <= threshold:
            return True
        else:
            return False
    
    def get_current_yaw(self):
        yaw_err = { '0'   : fabs(normalize_angle(fabs(-self.yaw))),
                    '90'  : fabs(normalize_angle(fabs(90. - self.yaw))),
                    '90N' : fabs(normalize_angle(fabs(-90. - self.yaw))),
                    '180' : fabs(normalize_angle(fabs(180 - self.yaw))) }

        yaw_err = sorted(yaw_err.items(),  key=lambda d: d[1], reverse=False)

        min_yaw_name = list(yaw_err[0])[0]
        if min_yaw_name == '0':
            return 0
        elif min_yaw_name == '90':
            return 90
        elif min_yaw_name == '90N':
            return -90
        elif min_yaw_name == '180':
            return 180
        else:
            return None

    def add_waypoint(self, x, y, yaw = None):
        # body frame in start, FLU
        # x, y in meters
        # yaw in degrees
        self.waypt.append([x, y, yaw])
        self.is_all_done = False