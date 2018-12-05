#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import random
from std_msgs.msg import Bool


NODE_NAME = 'movement_manager'


class MovementManager(object):
    def __init__(self):

        rospy.loginfo('[Movement Manager] Initializing communication with robot.')

        # Movement variables
        self.current_location = ''
        self.last_location = ''
        self.waypoints = # conversion of location description to (X,Y,Z) coordinates

        # Face Services and Topics
        self.animation_location_sub = # TODO
        self.animation_change_movement_point_sub = rospy.Subscriber('movement/point_movement',
                                                                    ChangeMovementPoint, self.handle_change_point)
        self.moving_robot_pub = rospy.Publisher('movement/moving_robot', Bool, queue_size=10000)

    def handle_change_location(self, msg):
        # send robot to point of location received
        # remember that at the end must publish that robot is moving and that movement is over when finishes

    def handle_change_point(self, msg):
        # send robot to point received
        # remember that at the end must publish that robot is moving



def main():
    rospy.init_node(NODE_NAME)
    rospy.loginfo('Starting Node: ' + NODE_NAME)
    imm = MovementManager()
    rospy.loginfo('Movement Manager node running')
    rospy.spin()


if __name__ == "__main__":
    main()
