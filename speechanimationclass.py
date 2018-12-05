#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import random
from trash_sentences import *
from std_msgs.msg import String


class SpeechAnimation(object):
    def __init__(self):
        rospy.loginfo('[Movement Manager] Initializing communication with robot.')

        # Movement variables
        self.used_sentences = dict()
        self.sentences = sentence1 # list of sentences by type
        rospy.loginfo(sentence1)

        # Face Services and Topics
        self.moving_robot_pub = rospy.Publisher('speech/speech_demo', String, queue_size=10000)

    def update_behavior(self):
        # what this module should do



