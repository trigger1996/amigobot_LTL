#!/usr/bin/env python
#encoding=utf-8

import sys
import math
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud

max_range     = 5.0 
min_angle     = -144
max_angle     = 144
deg_increment = 2
num_readings  = ((max_angle-min_angle) / deg_increment + 1)
number_of_sonar_sensors = 8

# Amigobot sonar directions in degrees
sonar_directions = [90, 44, 12, -12, -44, -90, -144, 144]

# p3dx sonar directions in degrees
#sonar_directions = [90, 67.5, 45, 22.5, -22.5, -45, -67.5, -90]



robot_name = str()


sonar_frequency = 25

class sonar_msg_transformer():
    def __init__(self, robot_name):
        self.sonar_sub      = rospy.Subscriber('/' + robot_name + '/sonar', PointCloud, self.sonar_cb)
        self.fake_laser_pub = rospy.Publisher('/' + robot_name + '/scan',   LaserScan, queue_size = 1)
        
        # Initialize sonar data with default values
        self.sonar_data = [1, 1, 1, 1, 1, 1, 1, 1]

    def sonar_cb(self, data):
        self.sonar_data = []
        for i in range(0, number_of_sonar_sensors):
            val = math.sqrt(data.points[i].x**2 + data.points[i].y**2)
            self.sonar_data.append(val)

        self.publishLaserFromSonar()

    def publishLaserFromSonar(self):
        ranges = []
        intensities = []
        # Generate empty data outside of the range for our laser scan
        for i in range(0, num_readings):
            ranges.append(float("inf"))      # max_range + 1.0
            intensities.append(0)


        # Generate actual sonar data in correct positions
        for i in range(0, number_of_sonar_sensors):
            index = (sonar_directions[i] - min_angle)/deg_increment
            ranges[index] = self.sonar_data[i]
            intensities[i] = 0.5

        msg_to_send = LaserScan()
        msg_to_send.header.frame_id = robot_name + "/base_link"
        msg_to_send.header.stamp = rospy.Time.now()

        msg_to_send.angle_min = min_angle * math.pi / 180.
        msg_to_send.angle_max = max_angle * math.pi / 180.
        msg_to_send.angle_increment = deg_increment * math.pi / 180.
        msg_to_send.time_increment  = (1.0 / sonar_frequency) / (num_readings)
        msg_to_send.range_min = 0.0
        msg_to_send.range_max = max_range
        for i in range(0, num_readings):
            msg_to_send.ranges.append(ranges[i])
            msg_to_send.intensities.append(intensities[i])

        self.fake_laser_pub.publish(msg_to_send)

def main():
    rospy.init_node('tf_transform', anonymous=False)

    robot_name = rospy.get_param('~robot_name', 'RosAria')

    sonar_msg_transformer(robot_name=robot_name)

    rospy.spin()

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass
