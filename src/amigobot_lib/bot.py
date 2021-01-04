#encoding=utf-8

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt

class amigobot(object):
    def __init__(self, name='RosAria'):
        self.name = name

        self.twist_pub = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size = 10)
        self.pose_sub  = rospy.Subscriber('/' + self.name + '/pose', Odometry, self.odom_cb)

        self.vx = 0
        self.wz = 0

        self.x = 0
        self.y = 0
        self.yaw = 0

        self.x_start = self.x
        self.y_start = self.y
        self.dist_desired = 0
        self.dist_curr = 0
        self.yaw_desired = 0

        self.basic_motion = [ 'standby', 'wait', 'forward', 'back', 'left_turn', 'right_turn']
        self.current_motion = 'standby'

    def odom_cb(self, data):
        # about 10Hz
        (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x, 
                                                    data.pose.pose.orientation.y, 
                                                    data.pose.pose.orientation.z, 
                                                    data.pose.pose.orientation.w])
        self.yaw = yaw * 180. / math.pi
        self.x   = data.pose.pose.position.x
        self.y   = data.pose.pose.position.y

        # set vel
        if self.current_motion == 'standby':
            self.set_vel(0, 0)

        elif self.current_motion == 'right_turn':
            self.set_vel(0, -math.pi / 10)
            if self.yaw <= self.yaw_desired:
                self.current_motion = 'standby'

        elif self.current_motion == 'left_turn':
            self.set_vel(0, math.pi / 10)
            if self.yaw >= self.yaw_desired:
                self.current_motion = 'standby'
        
        elif self.current_motion == 'forward':
            self.dist_curr = math.sqrt((self.x - self.x_start)**2 + (self.y - self.y_start)**2)
            
            self.set_vel(0.215, 0)                       # 0.48
            if self.dist_curr >= self.dist_desired:
                self.x_start = self.x
                self.y_start = self.y
                self.dist_curr = 0
                self.dist_desired = 0

                self.current_motion = 'standby'

        elif self.current_motion == 'back':
            self.dist_curr = math.sqrt((self.x - self.x_start)**2 + (self.y - self.y_start)**2)

            self.set_vel(-0.215, 0)                       # -0.48
            if self.dist_curr >= self.dist_desired:
                self.x_start = self.x
                self.y_start = self.y
                self.dist_curr = 0
                self.dist_desired = 0

                self.current_motion = 'standby'

        elif self.current_motion == 'wait':
            self.set_vel(0, 0)
            # send a sequence of wait signal to stop vehicle
            # to debug
            for i in range(0, 10):
                self.update_target_vel()
            self.current_motion = 'standby'

        self.update_target_vel()


    def update_target_vel(self):
        move_cmd = Twist()
        move_cmd.linear.x = self.vx
        move_cmd.angular.z =self.wz
        self.twist_pub.publish(move_cmd)

    def set_vel_w_update(self, vel, yaw):
        self.vx = vel
        self.wz = yaw

        move_cmd = Twist()
        move_cmd.linear.x = self.vx
        move_cmd.angular.z =self.wz
        self.twist_pub.publish(move_cmd)

    def set_vel(self, vel, yaw):
        self.vx = vel
        self.wz = yaw

    def turn_90_cw(self):
        self.yaw_desired = self.yaw - 90.
        self.current_motion = 'right_turn'

    def turn_90_ccw(self):
        self.yaw_desired = self.yaw + 90.
        self.current_motion = 'left_turn'

    def turn_cw(self, target_yaw):
        # in degree
        self.yaw_desired = self.yaw - target_yaw
        self.current_motion = 'right_turn'

    def turn_ccw(self, target_yaw):
        # in degree
        self.yaw_desired = self.yaw + target_yaw
        self.current_motion = 'left_turn'        

    def forward(self, dist):
        self.x_start = self.x
        self.y_start = self.y
        self.dist_desired = math.fabs(dist)
        self.current_motion = 'forward'

    def back(self, dist):
        self.x_start = self.x
        self.y_start = self.y
        self.dist_desired = math.fabs(dist)
        self.current_motion = 'back'

class amigobot_xyControl(amigobot):
    def __init__(self, name='RosAria'):
        super(amigobot_xyControl, self).__init__(name)

        self.route_list = []        # [[x1, y1], ..., [xn, yn]], "FLU" frame from start

        self.x_tgt = self.x 
        self.y_tgt = self.y
        self.yaw_to_turn = 0

        self.is_all_done = False    # added for while-loop

    def odom_cb(self, data):
        # update xy

        # target arrived
        if self.is_vertex_arrived(self.x_tgt, self.y_tgt):
            # with no next target
            if self.route_list.__len__() == 0:
                self.current_motion == 'standby'
                self.is_all_done = True
            # update next target
            else:
                self.x_tgt_last = self.x_tgt
                self.y_tgt_last = self.y_tgt
                [self.x_tgt, self.y_tgt] = self.route_list.pop(0)

                if self.x_tgt_last == self.x_tgt and self.y_tgt_last == self.y_tgt:
                    self.current_motion = 'wait'

                    self.x_tgt_last = self.x_tgt
                    self.y_tgt_last = self.y_tgt
                    [self.x_tgt, self.y_tgt] = self.route_list.pop(0)
                else:
                    # update target parameters
                    # yaw_to_turn
                    self.yaw_to_turn  = math.atan2(self.y_tgt - self.y, self.x_tgt - self.x) * 180. / math.pi
                    self.yaw_to_turn  = self.yaw_to_turn - self.yaw
                    # normalize to -180. - 180.
                    while self.yaw_to_turn >= 180.:
                        self.yaw_to_turn -= 360.
                    while self.yaw_to_turn < -180.:
                        self.yaw_to_turn += 360.
                    # target distance
                    self.dist_desired = math.sqrt((self.x_tgt - self.x)**2 + (self.y_tgt - self.y)**2)

        # target_not_arrived
        else:
            # first, align robot with target
            if self.is_angle_arrived() == False:
                if self.yaw_to_turn <= 0:
                    self.turn_cw(self.yaw_to_turn)
                else:
                    self.turn_ccw(self.yaw_to_turn)
            # second, go
            else:
                #self.forward(self.dist_desired)
                self.current_motion = 'forward'


        # update original data
        super(amigobot_xyControl, self).odom_cb(data)

        print('[Current]: ', self.name + ": (" + str([self.x_tgt, self.y_tgt]) + ", " + str([self.x, self.y]) + ")")
        #print('[Current]: ', self.name + ": (" + str(self.yaw_desired) + ", " + str(self.yaw_to_turn) + ", " + str(self.yaw) + ")")


    def is_angle_arrived(self, yaw_err = 9):
        # in degree
        target_yaw = math.atan2(self.y_tgt - self.y, self.x_tgt - self.x) * 180. / math.pi
        cur_yaw_err = self.yaw - target_yaw
        # normalization
        while cur_yaw_err >= 180.:
            cur_yaw_err -= 360.
        while cur_yaw_err < -180.:
            cur_yaw_err += 360.
        if math.fabs(cur_yaw_err) <= yaw_err:
            return True
        else:
            return False
    
    def is_vertex_arrived(self, x, y, err = 0.095):
        dist_err = math.sqrt((x - self.x)**2 + (y - self.y)**2)
        if math.fabs(dist_err) <= err:
            return True
        else:
            return False

    # in FLU frame for start point
    def add_waypoint(self, x, y):
        self.route_list.append([x, y])
        self.is_all_done = False

    def clear_waypoint(self):
        self.route_list.clear()


class amigobot_w_move_base(amigobot):
    def __init__(self, name='RosAria'):
        super(amigobot_xyControl, self).__init__(name)
        self.route_list = []

        rospy.init_node('MultiNav', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 10)

        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", False)

        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED',
                       'ABORTED', 'REJECTED', 'PREEMPTING', 'RECALLING',
                       'RECALLED', 'LOST']

        # Set up the goal locations. Poses are defined in the map frame.
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        locations = dict()

        # 替代为自己的地图上对应的位置，朝向默认为1

        locations['home_kitchen'] = Pose(Point(-1.03, 7.28, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))
        locations['home_hall'] = Pose(Point(-1.40, 4.30, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))
        locations['home_sofa'] = Pose(Point(-2.56, 2.82, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))
        locations['home_refrigerator'] = Pose(Point(-1.00, 6.88, 0.00), Quaternion(0.000, 0.000, 1.000, 0.000))
        locations['home_door'] = Pose(Point(-2.80, 8.00, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))
        locations['home_balcony'] = Pose(Point(-2.08, 4.57, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))

        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size = 5)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")

        # A variable to hold the initial pose of the robot to be set by the user in RViz
        initial_pose = PoseWithCovarianceStamped()
        # Variables to keep track of success rate, running time, and distance traveled
        n_locations = len(locations)
        n_goals = 0
        n_successes = 0
        i = n_locations
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        location = ""
        last_location = ""
        # Get the initial pose from the user
        rospy.loginfo("Click on the map in RViz to set the intial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.last_location = Pose()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
            rospy.sleep(1)
        rospy.loginfo("Starting navigation test")

        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():

            # If we've gone through the current sequence, start with a new random sequence
            if i == n_locations:
                i = 0
                sequence = sample(locations, n_locations)
                # Skip over first location if it is the same as the last location
                if sequence[0] == last_location:
                    i = 1

                    # Get the next location in the current sequence
            location = sequence[i]

            # Keep track of the distance traveled.
            # Use updated initial pose if available.
            if initial_pose.header.stamp == "":
                distance = sqrt(pow(locations[location].position.x
                                    - locations[last_location].position.x, 2) +
                                pow(locations[location].position.y -
                                    locations[last_location].position.y, 2))
            else:
                rospy.loginfo("Updating current pose.")
                distance = sqrt(pow(locations[location].position.x
                                    - initial_pose.pose.pose.position.x, 2) +
                                pow(locations[location].position.y -
                                    initial_pose.pose.pose.position.y, 2))
                initial_pose.header.stamp = ""

                # Store the last location for distance calculations
            last_location = location

            # Increment the counters
            i += 1
            n_goals += 1

            # Set up the next goal location
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = locations[location]
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()

            # Let the user know where the robot is going next
            rospy.loginfo("Going to: " + str(location))
            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)

            # Allow 5 minutes to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

            # Check for success or failure
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    n_successes += 1
                    distance_traveled += distance
                else:
                    rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))

                    # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0

            # Print a summary success/failure, distance traveled and time elapsed
            rospy.loginfo("Success so far: " + str(n_successes) + "/" +
                          str(n_goals) + " = " + str(100 * n_successes / n_goals) + "%")
            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
            rospy.sleep(self.rest_time)

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def trunc(f, n):
        # Truncates/pads a float f to n decimal places without rounding
        slen = len('%.*f' % (n, f))
        return float(str(f)[:slen])