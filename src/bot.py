#encoding=utf-8

import rospy
import math
from geometry_msgs.msg import Twist

class amigobot:
    def __init__(self, name='amigobot'):
        self.name = name

        self.twist_pub = rospy.Publisher('/' + name + '/cmd_vel', Twist, queue_size = 10)

        self.vx = 0
        self.wz = 0

    def set_vel(self, vx, wz):
        self.vx = vx
        self.wz = wz

        move_cmd = Twist()
        move_cmd.linear.x = self.vx
        move_cmd.angular.z =self.wz
        self.twist_pub.publish(move_cmd)

    def single_turn(self):
        self.set_vel(0., math.pi / 2.)

    def single_turn(self):
        self.set_vel(0., -math.pi / 2.)

    def single_forward(self):
        self.set_vel(0.5, 0)

    def single_back(self):
        self.set_vel(-0.5, 0)

    # motion primitives with costs #
    def mp_left_up(dist):
        self.set_vel(-0.5, 0)
