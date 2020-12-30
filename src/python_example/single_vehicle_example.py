#!/usr/bin/env python
#encoding=utf-8

import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('motion_primitive_test', anonymous=False)

    twist_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size = 10)
    rate = rospy.Rate(5)	# 5Hz

    while not rospy.is_shutdown():
        
        # https://www.cnblogs.com/shang-slam/p/6891086.html
        move_cmd = Twist()
        move_cmd.linear.x = 0.1
        move_cmd.angular.z = 0.3

        twist_pub.publish(move_cmd)
        rospy.sleep(1)


    print(233)

if __name__ == '__main__':
     try:
         main()
     except rospy.ROSInterruptException:
         pass
