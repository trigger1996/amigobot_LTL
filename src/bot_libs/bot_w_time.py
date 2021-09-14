#!/usr/bin/env python
#encoding=utf-8
import rospy
from geometry_msgs.msg import Twist, Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
import tf
from math import radians, copysign, sqrt, pow, pi, atan2, fabs, sin, cos
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformBroadcaster
import numpy as np

global ros_rate, u_dist_max
odom_cb_rate = 15       # read from gazebo, in hz, turtlebot3_waffle_pi.gazebo.xacro
u_dist_max = 0.185


def data_saturation(data, max, min):
    if data > max:
        return max
    elif data < min:
        return min
    else:
        return data

def normalize_angle(yaw):
    while yaw >= 180.:
        yaw -= 360.
    while yaw < -180.:
        yaw += 360.
    return yaw

class turtlebot(object):
    def __init__(self, name='amigobot_1', x = 0, y = 0, yaw = 0, time_to_wait = 1, u_dist_max = u_dist_max):
        self.name = name

        self.twist_pub        = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size = 1)
        self.pose_sub         = rospy.Subscriber('/' + self.name + '/odom', Odometry, self.odom_cb)

        self.robot_traj_pub   = rospy.Publisher('/' + self.name + '/path', Path, queue_size = 1)
        self.tf_baselink_odom = TransformBroadcaster()
        self.robot_traj = Path()

        self.x = x
        self.y = y
        self.yaw = yaw

        self.vx = 0
        self.wz = 0

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

        self.yaw_setpt_threshold  = 0.075                   # deg, ref: 5
        self.dist_setpt_threshold = 0.15                    # meter

        # yaw PI controller
        self.yaw_kp = 2.75
        self.yaw_ki = 0.15
        self.yaw_increment  = 0
        self.yaw_inc_max = 0.25
        self.u_yaw_max = 180                                # deg/s

        # yaw nonlinear controller
        self.yaw_c2 = 64.75
        self.yaw_c3 = 6.25

        # dist PI controller
        self.dist_kp = 4.25
        self.dist_ki = 0.2
        self.dist_increment  = 0
        self.dist_inc_max = 0.05
        self.u_dist_max = u_dist_max                        # m/s, maximum speed with no slide in startup: 0.3 (about)
        self.u_dist_desired = self.u_dist_max               # added for go-back reducing speed

        self.is_steer_completed = False
        self.is_wait = False                                # symbol for waiting

        self.time_to_wait = time_to_wait
        self.time_to_wait_errval =  1.5 / odom_cb_rate      # error value, theory value: 1. / odom_cb_rate, NEED TUNING

        # timer varibles
        self.timestamp_last = 0                             # timestamp for last motion compeletion
        self.timestamp_next = 0                             # timestamp for next motion
        self.time_curr = 0                                  # current time
        self.odom_cb_index = 0
        
        #
        self.init_time()

    def odom_cb(self, data):
        # about 120Hz
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

        if self.odom_cb_index % 12 == 0:
            self.publish_path_4_rviz()      # 10Hz

        self.odom_cb_index += 1
        if self.odom_cb_index > 1000:
            self.odom_cb_index = 0

    def motion_control(self):

        # if the maximum time is exceeded
        # goto next waypoint ASAP
        if self.timestamp_next != None and self.time_curr >= self.timestamp_next:

            # send a break
            self.update_target_vel(0, 0)
            self.is_steer_completed = False

            # no next points
            if self.waypt.__len__() == 0:

                self.is_all_done = True
            
            # next point exists
            else:

                print('[' + str(rospy.Time.now().secs) + " " + str(rospy.Time.now().nsecs) + '] ' + self.name + ' arrived: (' + str(self.x) + ', ' + str(self.y) + ')') # 'due to start / waiting / exceeding maximum time'

                self.target_x_last   = self.target_x
                self.target_y_last   = self.target_y
                self.target_yaw_last = self.target_yaw

                [self.target_x, self.target_y, self.target_yaw, self.target_t_max, self.u_dist_desired] = self.waypt.pop(0)

                if self.target_t_max == None:
                    self.timestamp_next = None
                else:
                    self.timestamp_next = self.time_curr + self.target_t_max - self.time_to_wait_errval

                if self.u_dist_desired == None:
                    self.u_dist_desired = self.u_dist_max
                self.u_dist_desired = data_saturation(self.u_dist_desired, self.u_dist_max, -self.u_dist_max)

                # if wait at the same point
                if self.target_x_last  == self.target_x and self.target_y_last == self.target_y:
                    
                    print('[' + str(rospy.Time.now().secs) + " " + str(rospy.Time.now().nsecs) + '] ' + self.name + ' waiting: (' + str(self.x) + ', ' + str(self.y) + ')')

                    self.is_wait = True
                    self.timestamp_next = self.time_curr + self.time_to_wait - self.time_to_wait_errval

                # else 
                else:
                    self.is_wait = False

        else:

            # obtain yaw error for final turn first
            yaw_err_final = None                                  # for desired yaw after arriving
            if self.target_yaw != None:
                yaw_err_final = normalize_angle(self.target_yaw - self.yaw)

            # if waiting
            if self.is_wait == True:

                self.update_target_vel(0, 0)

            else:

                # if arrived
                if self.is_vertex_arrived(self.target_x, self.target_y, self.dist_setpt_threshold) and \
                    (self.target_yaw == None or fabs(yaw_err_final) <=self.yaw_setpt_threshold):

                    # send a break
                    self.update_target_vel(0, 0)
                    self.is_steer_completed = False

                    # if arrived in advance
                    if self.timestamp_next != None and self.time_curr <= self.timestamp_next:

                        print('[' + str(rospy.Time.now().secs) + " " + str(rospy.Time.now().nsecs) + '] ' + self.name + ' arrived IN ADVANCE, waiting: (' + str(self.x) + ', ' + str(self.y) + ')')
                        
                        self.is_wait = True
                        return

                    # no next points
                    if self.waypt.__len__() == 0:
                       
                        self.is_all_done = True
                    
                    # next point exists
                    else:

                        print('[' + str(rospy.Time.now().secs) + " " + str(rospy.Time.now().nsecs) + '] ' + self.name + ' arrived: (' + str(self.x) + ', ' + str(self.y) + ')')

                        self.target_x_last   = self.target_x
                        self.target_y_last   = self.target_y
                        self.target_yaw_last = self.target_yaw

                        [self.target_x, self.target_y, self.target_yaw, self.target_t_max, self.u_dist_desired] = self.waypt.pop(0)

                        if self.u_dist_desired == None:
                            self.u_dist_desired = self.u_dist_max
                        self.u_dist_desired = data_saturation(self.u_dist_desired, self.u_dist_max, -self.u_dist_max)

                        if self.target_t_max == None:
                            self.timestamp_next = None
                        else:
                            self.timestamp_next = self.time_curr + self.target_t_max - self.time_to_wait_errval

                        # if waypoint is the same, wait
                        if self.target_x_last  == self.target_x and self.target_y_last == self.target_y:
                            
                            print('[' + str(rospy.Time.now().secs) + " " + str(rospy.Time.now().nsecs) + '] ' + self.name + ' waiting: (' + str(self.x) + ', ' + str(self.y) + ')')

                            self.is_wait = True
                            self.timestamp_next = self.time_curr + self.time_to_wait - self.time_to_wait_errval
        
                # target arrived but not turn to corresponding heading
                elif self.is_vertex_arrived(self.target_x, self.target_y, self.dist_setpt_threshold) and \
                    self.target_yaw != None and fabs(yaw_err_final) >= self.yaw_setpt_threshold:

                    yaw_err = normalize_angle(self.target_yaw - self.yaw)        # yaw_err_final

                    # PI control w saturation               
                    self.yaw_increment += yaw_err
                    self.yaw_increment = data_saturation(self.yaw_increment, self.yaw_inc_max, -self.yaw_inc_max)

                    u_yaw = self.yaw_kp * yaw_err + self.yaw_ki * self.yaw_increment
                    u_yaw = data_saturation(u_yaw, self.u_yaw_max, -self.u_yaw_max)

                    if fabs(yaw_err) <= self.yaw_setpt_threshold + 0.1:         # for easy to stop steering
                        self.is_steer_completed = True

                    self.update_target_vel(0, u_yaw * pi / 180)

                #
                else:

                    # calculate errors
                    yaw_err = atan2(self.target_y - self.y, self.target_x - self.x) * 180 / pi - self.yaw
                    yaw_err = normalize_angle(yaw_err)
                    dist_err = sqrt((self.target_y - self.y)**2 + (self.target_x - self.x)**2)            

                    # turn first
                    if fabs(yaw_err) >= self.yaw_setpt_threshold and self.is_steer_completed == False:
                        # PI control w saturation               
                        self.yaw_increment += yaw_err
                        self.yaw_increment = data_saturation(self.yaw_increment, self.yaw_inc_max, -self.yaw_inc_max)

                        u_yaw = self.yaw_kp * yaw_err + self.yaw_ki * self.yaw_increment
                        u_yaw = data_saturation(u_yaw, self.u_yaw_max, -self.u_yaw_max)

                        if fabs(yaw_err) <= self.yaw_setpt_threshold + 0.1:         # for easy to stop steering
                            self.is_steer_completed = True

                        self.update_target_vel(0, u_yaw * pi / 180)

                    else:

                        # PI control w saturation
                        # dist                      
                        self.dist_increment += dist_err
                        self.dist_increment = data_saturation(self.dist_increment, self.dist_inc_max, -self.dist_inc_max)

                        u_dist = self.dist_kp * dist_err + self.dist_ki * self.dist_increment
                        u_dist = data_saturation(u_dist, self.u_dist_max, -self.u_dist_max)

                        # yaw P controller
                        u_yaw = self.yaw_kp * yaw_err
                        u_yaw = data_saturation(u_yaw, self.u_yaw_max, -self.u_yaw_max)


                        # controller from Yu 2015
                        '''
                        vr = 0
                        wr = 0

                        xe = (self.target_x - self.x) *  cos(self.yaw * pi / 180) + (self.target_y - self.y) * sin(self.yaw * pi / 180)
                        ye = (self.target_x - self.x) * -sin(self.yaw * pi / 180) + (self.target_y - self.y) * cos(self.yaw * pi / 180)

                        self.dist_increment += xe
                        self.dist_increment = data_saturation(self.dist_increment, self.dist_inc_max, -self.dist_inc_max)

                        u_dist = vr + self.dist_kp * xe + #self.dist_ki * self.dist_increment
                        u_yaw  = wr + self.yaw_c2 * (ye * cos(yaw_err / 2 * pi / 180) - xe * sin(yaw_err / 2 * pi / 180)) + self.yaw_c3 * sin(yaw_err / 2 * pi / 180)

                        u_dist = data_saturation(u_dist, self.u_dist_desired, -self.u_dist_desired)
                        u_yaw = data_saturation(u_yaw, self.u_yaw_max, -self.u_yaw_max)
                        '''
                        #
                        self.update_target_vel(u_dist, u_yaw * pi / 180 * 0.066)
                        #self.update_target_vel(u_dist, 0)
            

    def update_target_vel(self, v, w):
        self.vx = v
        self.wz = w

        move_cmd = Twist()
        move_cmd.linear.x = self.vx
        move_cmd.angular.z =self.wz
        self.twist_pub.publish(move_cmd)

    def publish_tf_4_rviz(self):
        self.tf_baselink_odom.sendTransform((self.x, self.y, 0),
                                           tf.transformations.quaternion_from_euler(0, 0, self.yaw * pi / 180.),
                                           rospy.Time.now(),
                                           self.name + "/base_link",
                                           "map")
    
    def publish_path_4_rviz(self):
            bot_pose_t = PoseStamped()

            bot_pose_t.header.stamp = rospy.Time.now()
            bot_pose_t.header.frame_id = '/amigobot_1/odom'
            bot_pose_t.pose.position.x = self.x
            bot_pose_t.pose.position.y = self.y
            bot_pose_t.pose.position.z = 0
            [bot_pose_t.pose.orientation.x, bot_pose_t.pose.orientation.y, bot_pose_t.pose.orientation.z, bot_pose_t.pose.orientation.w] = quaternion_from_euler(0., 0., self.yaw * pi / 180.)

            self.robot_traj.poses.append(bot_pose_t)
            self.robot_traj.header.stamp = rospy.Time.now()
            self.robot_traj.header.frame_id = '/amigobot_1/odom'
            self.robot_traj_pub.publish(self.robot_traj)

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
        dist = sqrt((self.x - x) ** 2 + (self.y - y) ** 2)
        if dist <= threshold:
            return True
        else:
            return False

    def add_waypoint(self, x, y, yaw = None, maximum_time=None, desired_vel=None):
            self.waypt.append([x, y, yaw, maximum_time, desired_vel])
            self.is_all_done = False