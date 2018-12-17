#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import os
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Bool
import random


NODE_NAME = 'remote_movement_manager'


class RemoteMovementManager(object):
    def __init__(self):

        rospy.loginfo('[Remote Movement Manager] Initializing remote remote movement listener node.')

        # Face Services and Topics
        self.animation_movement_sub = rospy.Subscriber('movement/point_movement', 
                                                                    String, self.handle_change_point)
        self.animation_stop_sub = rospy.Subscriber('navigation/stop_moving', 
                                                                    String, self.handle_stop)
        self.moving_robot_pub = rospy.Publisher('movement/moving_robot', Bool, queue_size=10000)

    def handle_change_point(self, msg):
        print('Received movement message: ' + msg)

    def handle_stop(self, msg):
        print('Received stop command!')
        
def main():
    rospy.init_node(NODE_NAME)
    rospy.loginfo('Starting Node: ' + NODE_NAME)
    rmm = RemoteMovementManager()
    rospy.loginfo('Remote Movement Manager node running')
    rospy.loginfo('Your network card config: \n' + os.popen('ip addr show').read())
    rospy.spin()


if __name__ == "__main__":
    main()
