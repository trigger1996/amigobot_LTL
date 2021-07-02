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

        self.twist_pub        = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size = 1)
        self.pose_sub         = rospy.Subscriber('/' + self.name + '/odom', Odometry, self.odom_cb)

        self.robot_traj_pub   = rospy.Publisher('/' + self.name + '/path', Path, queue_size = 1)

        self.tf_baselink_odom = TransformBroadcaster()
        self.tf_map_odom = TransformListener()

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