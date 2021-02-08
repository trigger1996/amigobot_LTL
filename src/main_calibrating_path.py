#!/usr/bin/env python
#encoding=utf-8

import sys
sys.path.append("/home/ghost/catkin_ws_ros/src/amigobot_LTL/src/LOMAP_Custom/")               # root path: amigobot_LTL/src

import rospy
import bot_libs.bot as bot
import bot_libs.bot_ts as bot_ts

from math import pi
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler

global time_to_wait
time_to_wait = 10        # seconds

def main():
    rospy.init_node('ijrr2013_ca_improv', anonymous=False)

    rate = rospy.Rate(60)	# 60Hz

    rospy.sleep(8)

 
    bot_1 = bot_ts.turtlebot_TS(name='amigobot_1', yaml_file='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/robot_1.yaml',
                                                   map_file ='/home/ghost/catkin_ws_ros/src/amigobot_LTL/model/ijrr_2013_improv/map.yaml',
                                                   time_to_wait = time_to_wait)

    robot_traj_pub = rospy.Publisher('/amigobot_1/path', Path, queue_size = 1)
    robot_traj = Path()
    index = 0

    '''
    # outer loop
    bot_1.add_waypoint_from_waypt_list('4')
    bot_1.add_waypoint_from_waypt_list('4')
    bot_1.add_waypoint_from_waypt_list('5')
    bot_1.add_waypoint_from_waypt_list('6')
    bot_1.add_waypoint_from_waypt_list('7')
    bot_1.add_waypoint_from_waypt_list('8')
    bot_1.add_waypoint_from_waypt_list('9')
    bot_1.add_waypoint_from_waypt_list('10')
    bot_1.add_waypoint_from_waypt_list('u2')
    bot_1.add_waypoint_from_waypt_list('10')   
    bot_1.add_waypoint_from_waypt_list('11')
    bot_1.add_waypoint_from_waypt_list('12')
    bot_1.add_waypoint_from_waypt_list('1')
    bot_1.add_waypoint_from_waypt_list('2')
    bot_1.add_waypoint_from_waypt_list('3')
    bot_1.add_waypoint_from_waypt_list('4')
    bot_1.add_waypoint_from_waypt_list('u1')    
    bot_1.add_waypoint_from_waypt_list('u1')    # for showing arriving time
    '''
    
    
    # inner loop
    bot_1.add_waypoint_from_waypt_list('4')
    bot_1.add_waypoint_from_waypt_list('4')
    bot_1.add_waypoint_from_waypt_list('5')
    bot_1.add_waypoint_from_waypt_list('27')
    bot_1.add_waypoint_from_waypt_list('28')
    bot_1.add_waypoint_from_waypt_list('g4')
    bot_1.add_waypoint_from_waypt_list('28')
    bot_1.add_waypoint_from_waypt_list('21')
    bot_1.add_waypoint_from_waypt_list('22')
    bot_1.add_waypoint_from_waypt_list('g1')
    bot_1.add_waypoint_from_waypt_list('22')
    bot_1.add_waypoint_from_waypt_list('23')
    bot_1.add_waypoint_from_waypt_list('24')
    bot_1.add_waypoint_from_waypt_list('g2')
    bot_1.add_waypoint_from_waypt_list('24')
    bot_1.add_waypoint_from_waypt_list('25')
    bot_1.add_waypoint_from_waypt_list('26')
    bot_1.add_waypoint_from_waypt_list('g3')
    bot_1.add_waypoint_from_waypt_list('26')
    bot_1.add_waypoint_from_waypt_list('27')
    bot_1.add_waypoint_from_waypt_list('3')    
    bot_1.add_waypoint_from_waypt_list('4')
    bot_1.add_waypoint_from_waypt_list('u1')    
    bot_1.add_waypoint_from_waypt_list('u1')    # for showing arriving time

    '''
    # etc
    bot_1.add_waypoint_from_waypt_list('4')
    bot_1.add_waypoint_from_waypt_list('4')
    bot_1.add_waypoint_from_waypt_list('5')
    bot_1.add_waypoint_from_waypt_list('27')
    bot_1.add_waypoint_from_waypt_list('28')
    bot_1.add_waypoint_from_waypt_list('21')
    bot_1.add_waypoint_from_waypt_list('12')
    bot_1.add_waypoint_from_waypt_list('11')    # cheating, inverted
    bot_1.add_waypoint_from_waypt_list('23')
    bot_1.add_waypoint_from_waypt_list('24')
    bot_1.add_waypoint_from_waypt_list('25')
    bot_1.add_waypoint_from_waypt_list('6')
    bot_1.add_waypoint_from_waypt_list('5')
    bot_1.add_waypoint_from_waypt_list('27')
    bot_1.add_waypoint_from_waypt_list('3')    
    bot_1.add_waypoint_from_waypt_list('4')
    bot_1.add_waypoint_from_waypt_list('u1')    
    bot_1.add_waypoint_from_waypt_list('u1')    # for showing arriving time
    '''

    while not rospy.is_shutdown():
        if index >= 5:
            bot_pose_t = PoseStamped()
            bot_pose_t.header.stamp = rospy.Time.now()
            bot_pose_t.header.frame_id = '/amigobot_1/odom'
            bot_pose_t.pose.position.x = bot_1.x
            bot_pose_t.pose.position.y = bot_1.y
            bot_pose_t.pose.position.z = 0
            [bot_pose_t.pose.orientation.x, bot_pose_t.pose.orientation.y, bot_pose_t.pose.orientation.z, bot_pose_t.pose.orientation.w] = quaternion_from_euler(0., 0., bot_1.yaw * pi / 180.)

            robot_traj.poses.append(bot_pose_t)
            robot_traj.header.stamp = rospy.Time.now()            
            robot_traj.header.frame_id = '/amigobot_1/odom'
            robot_traj_pub.publish(robot_traj)

            index = 0

        index += 1
        rate.sleep()

    print("Finished!")

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass