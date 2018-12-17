#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import random
import geometry_msgs.msg
from std_msgs.msg import Bool, UInt32, String

NODE_NAME = 'remote_movement_controller'


class RemoteMovement(object):
    def __init__(self):
        rospy.loginfo('[Remote movement controller] Starting the remote control node.')

        # Variables

        ###################
        # Movement Topics #
        ###################

        # publishes location to move towards
        self.animation_move_location = rospy.Publisher('movement/point_movement', String, queue_size=100000)

        # stops robot (Emergency use)
        self.stop_moving = rospy.Publisher('navigation/stop_moving', Bool, queue_size=100000)

    def handle_is_moving_update(self, msg):
        self.is_moving = msg.data

    def update_behavior(self, msg):
        print("Sent msg: " + msg)
        if(msg == 'xxx'):
            self.stop_moving.publish(msg)
        else:
            self.animation_move_location.publish(msg)

def main():
    rospy.init_node(NODE_NAME)
    rospy.loginfo('Starting Node: ' + NODE_NAME)
    rm = RemoteMovement()
    rospy.loginfo('[REMOTE MOVEMENT] Node running')
    #master_ip = raw_input('Input the ROS MASTER IP address: ')
    #os.system('export ROS_MASTER_URI=http://' + str(master_ip) + ':11311')
    #rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        msg_key = str(raw_input('Type message key: '))
        rm.update_behavior(msg_key)

if __name__ == "__main__":
    main()
