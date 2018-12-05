#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import random
from trash_sentences import *
from std_msgs.msg import String

NODE_NAME = 'speech_animation'

class SpeechAnimation(object):
    def __init__(self):
        rospy.loginfo('[Movement Manager] Initializing communication with robot.')

        # Movement variables
        #self.used_sentences = dict()
        self.sentences = {
            1:sentence1,
            2:sentence2,
            3:sentence3
        } 
        rospy.loginfo(self.sentences[1])

        # Face Services and Topics
        self.moving_robot_pub = rospy.Publisher('speech/speech_demo', String, queue_size=10000)

    def update_behavior(self):
        # what this module should do
        rospy.loginfo("test")

# send robot to point received
# remember that at the end must publish that robot is moving
def main():
    rospy.init_node(NODE_NAME)
    rospy.loginfo('Starting Node: ' + NODE_NAME)
    spanim = SpeechAnimation()
    rospy.loginfo('Movement Manager node running')
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        spanim.update_behavior()
        rate.sleep()

if __name__ == "__main__":
    main()