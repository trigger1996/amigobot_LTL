#!/usr/bin/env python
#encoding=utf-8

import rospy
from geometry_msgs.msg import Twist, Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, fabs, sin, cos
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformBroadcaster, TransformListener
import numpy as np

import io
from bot_ts import turtlebot_TS

goback_additional_time = 10    # seconds

class amigobot_TS(turtlebot_TS):
    def __init__(self, name='amigobot_1', x = None, y = None, yaw = None, time_to_wait = 1, u_dist_max = 0.2, yaml_file='robot.yaml', map_file='map.yaml'):
        super(amigobot_TS, self).__init__(name, x, y, yaw, time_to_wait, u_dist_max, yaml_file, map_file)

        self.tf_map_odom = TransformListener()

        '''
            Parameters can be tuned here only for physical setup
        '''
        # waypoint varibles
        self.target_x = self.x                              # current target
        self.target_y = self.y
        self.target_yaw = None
        self.target_t_max = None
        self.target_x_last = None
        self.target_y_last = None
        self.target_yaw_last = None
        self.waypt = []                                     # next target list [[x, y, yaw, maximum_time]]

        self.is_all_done = False

        # v-w controller target
        self.yaw_setpoint = 0
        #self.dist_setpoint = 0

        self.yaw_setpt_threshold  = 0.15                    # deg, ref: 5
        self.dist_setpt_threshold = 0.15                    # meter

        # yaw PI controller
        self.yaw_kp = 2.75
        self.yaw_ki = 0.15
        self.yaw_increment  = 0
        self.yaw_inc_max = 0.25
        self.u_yaw_max = 180                                # deg/s

        # yaw nonlinear controller
        self.yaw_c2 = 18.75
        self.yaw_c3 = 6.25

        # dist PI controller
        self.dist_kp = 0.85
        self.dist_ki = 0.2
        self.dist_increment  = 0
        self.dist_inc_max = 0.2
        self.u_dist_max = u_dist_max                        # m/s, maximum speed with no slide in startup: 0.3 (about)
        self.u_dist_desired = self.u_dist_max               # added for go-back reducing speed

        self.is_steer_completed = False
        self.is_wait = False                                # symbol for waiting

        self.time_to_wait = time_to_wait
        self.time_to_wait_errval =  1.5 / odom_cb_rate      # error value, theory value: 1. / odom_cb_rate, NEED TUNING

    def odom_cb(self, data):

        (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x, 
                                                    data.pose.pose.orientation.y, 
                                                    data.pose.pose.orientation.z, 
                                                    data.pose.pose.orientation.w])
        self.yaw = yaw * 180. / pi
        self.x   = data.pose.pose.position.x
        self.y   = data.pose.pose.position.y

        try:
            # obtain transform
            (trans,rot) = self.tf_map_odom.lookupTransform('/map', '/' + self.name + '/odom', rospy.Time(0))
            (roll_tf, pitch_tf, yaw_tf) = euler_from_quaternion(rot)
            x_tf = trans[0]
            y_tf = trans[1]

            x_t = self.x *  cos(-yaw_tf) + self.y * sin(-yaw_tf)
            y_t = self.x * -sin(-yaw_tf) + self.y * cos(-yaw_tf)

            self.x   = x_t + x_tf
            self.y   = y_t + y_tf
            self.yaw = self.yaw + yaw_tf * 180. / pi

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('[Warning] No TF found for ' + self.name)
            pass

        self.update_time()
        self.motion_control()
        #self.publish_tf_4_rviz()

        if self.odom_cb_index % 12 == 0:
            self.publish_path_4_rviz()      # 10Hz

        self.odom_cb_index += 1
        if self.odom_cb_index > 1000:
            self.odom_cb_index = 0
        
    def publish_tf_4_rviz(self):
        pass
    
    def publish_path_4_rviz(self):
            bot_pose_t = PoseStamped()

            bot_pose_t.header.stamp = rospy.Time.now()
            bot_pose_t.header.frame_id = '/map'
            bot_pose_t.pose.position.x = self.x
            bot_pose_t.pose.position.y = self.y
            bot_pose_t.pose.position.z = 0
            [bot_pose_t.pose.orientation.x, bot_pose_t.pose.orientation.y, bot_pose_t.pose.orientation.z, bot_pose_t.pose.orientation.w] = quaternion_from_euler(0., 0., self.yaw * pi / 180.)

            self.robot_traj.poses.append(bot_pose_t)
            self.robot_traj.header.stamp = rospy.Time.now()
            self.robot_traj.header.frame_id = '/map'
            self.robot_traj_pub.publish(self.robot_traj)