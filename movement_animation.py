#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import random
import geometry_msgs.msg
from std_msgs.msg import Bool, UInt32

NODE_NAME = 'movement_animation'


class MovementAnimation(object):
    def __init__(self):
        rospy.loginfo('[Movement Animation] Initializing communication with robot.')

        # Variables
        self.is_moving = False
        self.last_points = []

        ###################
        # Movement Topics #
        ###################

        # controls if robot is moving
        self.moving_robot_pub = rospy.Subscriber('movement/moving_robot', Bool, self.handle_is_moving_update)

        # publishes point to move towards
        #   ChangeMovementPoint is a message that doesen't exist, you can either create it or check the ros geomety_msgs for a message that use (X,Y,Z)
        self.animation_change_movement_point_pub = rospy.Publisher('movement/point_movement', geometry_msgs.msg.Point, queue_size=100000)

        # publishes location to move towards
        self.animation_move_location = rospy.Publisher('movement/location_movement', String, queue_size=100000)

        # stops robot (Emergency use)
        self.stop_moving = rospy.Publisher('navigation/stop_moving', Bool, queue_size=100000)

    def handle_is_moving_update(self, msg):
        self.is_moving = msg.data

    def update_behavior(self, msg):
        # what this module should do
        # in "idle mode" picks a random point to move towards
        # in "controlled mode" doesn't do anything
        print(msg)
        if isinstance(msg, tuple):
            self.animation_change_movement_point_pub.publish(msg)
        else:
            self.animation_move_location.publish(msg)

def main():
    rospy.init_node(NODE_NAME)
    rospy.loginfo('Starting Node: ' + NODE_NAME)
    icm = MovementAnimation()
    rospy.loginfo('[MOVEMENT ANIMATION] Node running')
    #rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        msg_key = input("Type message key: ")
        icm.update_behavior(str(msg_key))

if __name__ == "__main__":
    main()
