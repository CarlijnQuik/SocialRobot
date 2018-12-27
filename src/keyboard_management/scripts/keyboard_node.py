#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool
import random

NODE_NAME = 'keyboard_manager'


class KeyboardManager(object):
    def __init__(self):
        rospy.loginfo('[Movement Manager] Initializing communication with robot.')

        self.moving_robot_pub = rospy.Publisher('joy', Twist, queue_size=10000)

    def update_behavior(self, msg):
        print(msg)
        vel_msg = Twist()
	speedList = [0, 0.30]
        vel_msg.angular.z = 0
        vel_msg.linear.x = random.choice(speedList)
        self.moving_robot_pub.publish(vel_msg)

def main():
    rospy.init_node(NODE_NAME)
    rospy.loginfo('Starting Node: ' + NODE_NAME)
    kbm = KeyboardManager()
    rospy.loginfo('Keyboard Manager node running')

    while not rospy.is_shutdown():
        msg_key = raw_input("Press space to move: ")
        kbm.update_behavior(str(msg_key))

    #rospy.spin()


if __name__ == "__main__":
    main()
