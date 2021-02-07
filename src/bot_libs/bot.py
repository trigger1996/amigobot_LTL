#!/usr/bin/env python
#encoding=utf-8
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, fabs
from tf.transformations import euler_from_quaternion
from tf import TransformBroadcaster
import numpy as np

global ros_rate
odom_cb_rate = 120       # read from gazebo, in hz

global motion_ref
motion_ref = ['right_turn', 'left_turn', 'u_turn', 'forward', 'wait', 'standby']

class turtlebot(object):
    def __init__(self, name='amigobot_1', model=None, x = 0, y = 0, yaw = 0, time_to_wait = 1):
        self.name = name

        self.twist_pub = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size = 1)
        self.pose_sub  = rospy.Subscriber('/' + self.name + '/odom', Odometry, self.odom_cb)

        self.tf_basefootprint_odom = TransformBroadcaster()        

        self.x = x
        self.y = y
        self.yaw = yaw

        self.vx = 0
        self.wz = 0

        # waypoint varibles
        self.target_x = self.x      # current target
        self.target_y = self.y
        self.target_yaw = None
        self.target_x_last = None
        self.target_y_last = None
        self.target_yaw_last = None
        self.waypt = []             # next target list [[x, y, yaw]]

        self.is_all_done = False

        # timer varibles
        self.timestamp_last = 0     # timestamp for last motion compeletion
        self.timestamp_next = 0     # timestamp for next motion
        self.time_curr = 0          # current time
        self.init_time()
       
        # wait varibles
        self.current_motion = 'standby'
        self.target_duration = 0.


        # control lib varibles
        self.motion_duration = {'right_turn' : 2.,                         # should be tuned with POSE in GAZEBO
                                'left_turn'  : 2.,
                                'u_turn'     : 4.,
                                'forward'    : None,
                                'wait'       : None,
                                'standby'    : None}
        self.motion_speed    = {'right_turn' : [0., -pi / 4 * 1.10],       # [vx, wz]
                                'left_turn'  : [0.,  pi / 4 * 1.10],
                                'u_turn'     : [0.,  pi / 4 * 1.05],
                                'forward'    : [0.5, 0.],
                                'wait'       : [0.,  0.],
                                'standby'    : [0.,  0.]}


    def odom_cb(self, data):
        # about 10Hz
        (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x, 
                                                    data.pose.pose.orientation.y, 
                                                    data.pose.pose.orientation.z, 
                                                    data.pose.pose.orientation.w])
        self.yaw = yaw * 180. / pi
        self.x   = data.pose.pose.position.x
        self.y   = data.pose.pose.position.y

        self.update_time()
        self.motion_control()
        self.publish_tf_4_rviz()

    def motion_control(self):

        # if time ended, pop next motion out

        if self.is_all_done == True:
            self.timestamp_last = self.timestamp_next

            if self.current_motion == 'forward':
                self.vx = self.motion_speed['forward'][0]
                self.wz = self.motion_speed['forward'][1]            
                self.timestamp_next = self.time_curr + self.target_duration

                self.is_all_done = False
            elif self.current_motion == 'right_turn':
                self.vx = self.motion_speed['right_turn'][0]
                self.wz = self.motion_speed['right_turn'][1]            
                self.timestamp_next = self.time_curr + self.motion_duration['right_turn']

                self.is_all_done = False
            elif self.current_motion == 'left_turn':
                self.vx = self.motion_speed['left_turn'][0]
                self.wz = self.motion_speed['left_turn'][1]            
                self.timestamp_next = self.time_curr + self.motion_duration['left_turn']

                self.is_all_done = False
            elif self.current_motion == 'u_turn':
                self.vx = self.motion_speed['u_turn'][0]
                self.wz = self.motion_speed['u_turn'][1]            
                self.timestamp_next = self.time_curr + self.motion_duration['u_turn']

                self.is_all_done = False
            elif self.current_motion == 'wait':
                self.vx = self.motion_speed['wait'][0]
                self.wz = self.motion_speed['wait'][1]            
                self.timestamp_next = self.time_curr + self.target_duration

                self.is_all_done = False
            else:
                self.vx = self.motion_speed['standby'][0]
                self.wz = self.motion_speed['standby'][1]            
                self.timestamp_next = self.time_curr

                self.is_all_done = True

        if self.timestamp_next >= self.time_curr:
            self.update_target_vel(self.vx, self.wz)
        else:
            self.current_motion = 'standby'

            self.vx = self.motion_speed['standby'][0]
            self.wz = self.motion_speed['standby'][1]              

            self.update_target_vel(self.vx, self.wz)
            self.is_all_done = True

    def update_target_vel(self, v, w):
        move_cmd = Twist()
        move_cmd.linear.x = v
        move_cmd.angular.z =w
        self.twist_pub.publish(move_cmd)

    def publish_tf_4_rviz(self):
        self.tf_basefootprint_odom.sendTransform((self.x, self.y, 0),
                                           tf.transformations.quaternion_from_euler(0, 0, self.yaw * pi / 180.),
                                           rospy.Time.now(),
                                           self.name + "/base_link",
                                           "map")
        
    def init_time(self):
        t_sec  = rospy.Time.now().secs
        t_nsec = rospy.Time.now().nsecs
        self.timestamp_last = float(t_sec) + float(t_nsec) / 1.e9
        self.timestamp_next = self.timestamp_last
        self.time_curr      = self.timestamp_last

    def update_time(self):
        t_sec  = rospy.Time.now().secs
        t_nsec = rospy.Time.now().nsecs
        self.time_curr = float(t_sec) + float(t_nsec) / 1.e9

    def is_vertex_arrived(self, x, y, threshold = 0.05):
        pass
        '''
        dist = sqrt((self.x - x) ** 2 + (self.y - y) ** 2)
        if dist <= threshold:
            return True
        else:
            return False
        '''
    def add_motion(self, motion, duration = 1):
        #self.current_motion = 
        pass
        #motion_ref = ['right_turn', 'left_turn', 'u_turn', 'forward', 'wait', 'standby']

    def add_waypoint(self, x, y, yaw = None):
        pass

