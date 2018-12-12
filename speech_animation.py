#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import random
from trash_sentences import *
from std_msgs.msg import String

NODE_NAME = 'speech_animation'

class SpeechAnimation(object):
    def __init__(self):
        rospy.loginfo('[Speech Manager] Initializing communication with robot.')

        # Movement variables
        # self.used_sentences = dict()
        self.sentences = {"1":sentence1, "2":sentence2, "3":sentence3 } 
        rospy.loginfo(self.sentences["1"])
        rospy.loginfo(self.sentences["2"])
        rospy.loginfo(self.sentences["3"])

        # Face Services and Topics
        self.speaking_robot_pub = rospy.Publisher('speech/speech_demo', String, queue_size=10000)

    def update_behavior(self, messageKey):
        # what this module should do
	    self.speaking_robot_pub.publish(self.sentences[messageKey])

# send robot to point received
# remember that at the end must publish that robot is moving
def main():
    rospy.init_node(NODE_NAME)
    rospy.loginfo('Starting Node: ' + NODE_NAME)
    spanim = SpeechAnimation()
    rospy.loginfo('Speech Manager node running')
    #rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        msg_key = input("Type message key: ")
        spanim.update_behavior(str(msg_key))
        #rate.sleep()

if __name__ == "__main__":
    main()
