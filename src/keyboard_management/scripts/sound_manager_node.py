#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import random
import numpy as np
from std_msgs.msg import String

NODE_NAME = 'sound_manager'


def speech_handler(msg):
    sentence = msg.data
    exe_command = 'espeak -s 175 -p 90 "' + sentence + '"'
    os.system(exe_command)
    rospy.loginfo("[SPEECH MANAGER]: Said: " + msg.data)


def main():
    rospy.loginfo('Starting Sound Manager')
    rospy.init_node(NODE_NAME)
    rospy.Subscriber('speech/speech_reader', String, speech_handler)
    rospy.loginfo('Sound Manager')
    rate = rospy.Rate(5)
    rospy.spin()


if __name__ == '__main__':
    main()
