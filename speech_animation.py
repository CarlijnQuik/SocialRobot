#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import random
from trash_sentences import *
from std_msgs.msg import String
from speechanimationclass import SpeechAnimation

NODE_NAME = 'speech_animation'

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