#!/usr/bin/env python
# -*- coding: utf-8 -*-

import curses
import time
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from trash_sentences import *
from std_msgs.msg import Bool, Int16
import random

NODE_NAME = 'experiment_controller'

POINT_A = "A"
POINT_B = "B"
POINT_C = "C"
POINT_D = "D"


class ExperimentController(object):
    checkpointList = [POINT_D, POINT_C, POINT_B, POINT_A]
    checkpointTimeDictionary = {
        POINT_A: 10,
        POINT_B: 9,
        POINT_C: 7,
        POINT_D: 8.5
    }

    active_sound_key_list = ["1", "2", "3", "4", "5"]
    thank_sound_key_list = ["9", "0"]

    nextSoundToPlay = ""

    ROTATION_LEFT_90 = "RL90"
    ROTATION_RIGHT_90 = "RR90"

    activeRotation = ROTATION_RIGHT_90

    def __init__(self):
        rospy.loginfo('[Experiment Controller] Initializing communication with managers.')

        # Movement management
        self.moving_robot_pub = rospy.Publisher('joy', Twist, queue_size=10000)
        self.movement_state_pub = rospy.Publisher('movement_state', Bool, queue_size=10)
        self.movement_angle_pub = rospy.Publisher('movement_angle', Int16, queue_size=1000)

        # Sound management
        self.sentences = {"1": sentence1, "2": sentence2, "3": sentence3, "4": sentence4, "5": sentence5,
                          "9": sentence9, "0": sentence0}
        self.moving_robot_pub = rospy.Publisher('speech/speech_reader', String, queue_size=10000)

    def play_sound(self, sound_key):
        self.moving_robot_pub.publish(self.sentences[sound_key])

    def update_angle(self, angleCommand):
        if (angleCommand == "a"):
            self.movement_angle_pub.publish(7.5)
        elif (angleCommand == "d"):
            self.movement_angle_pub.publish(-7.5)
        elif (angleCommand == self.ROTATION_LEFT_90):
            self.movement_angle_pub.publish(70)
        elif (angleCommand == self.ROTATION_RIGHT_90):
            self.movement_angle_pub.publish(-70)

    def update_movement(self, isMoving):
        self.movement_state_pub.publish(isMoving)

    def handle_next_step(self):
        if (self.checkpointList == []):
            rospy.loginfo('No points left to move. To reset please press >>>b<<<')
        else:
            nextStep = self.checkpointList.pop()
            timeToNextCheckpoint = self.checkpointTimeDictionary[nextStep]
            self.movement_state_pub.publish(True)
            # Put this in a while with time check instead of sleep the thread
            time.sleep(timeToNextCheckpoint)
            self.movement_state_pub.publish(False)
            self.play_sound(self.nextSoundToPlay)
            time.sleep(10)
            self.update_angle(self.activeRotation)

    def reset_state(self):
        self.checkpointList = [POINT_D, POINT_C, POINT_B, POINT_A]

    def handle_input(self, input_message):
        if input_message == "a" or input_message == "d":
            self.update_angle(input_message)
        elif input_message == "w":
            self.update_movement(True)
        elif input_message == "s":
            self.update_movement(False)
            self.update_angle(0)
        elif input_message in self.active_sound_key_list:
            self.nextSoundToPlay = input_message
            self.handle_next_step()
        elif input_message in self.thank_sound_key_list:
            self.play_sound(input_message)
        elif input_message == "b":
            self.reset_state()
        elif input_message == "t":
            curses.nocbreak()
            self.stdscr.keypad(False)
            curses.echo()
            curses.endwin()
        elif input_message == "c":
            self.invert_active_direction()
        elif input_message == "h":
            rospy.loginfo('Controll keys:\n a - rotate left \n d - rotate right \n w - move forward \n s - stop \n c - invert direction \n\n Play sounds: 1,2,3,4, \n Play thank you: 9 \n Play Nice job: 0 \n \n Utils: h - for help\n b - reset script \n t - clean the screen\n')
        else:
            rospy.loginfo("Invalid command. Press h for help.")

    def invert_active_direction(self):
        if self.activeRotation == self.ROTATION_LEFT_90:
            self.activeRotation = self.ROTATION_RIGHT_90
        else:
            self.activeRotation = self.ROTATION_LEFT_90
        self.update_angle(self.activeRotation)

def main():
    rospy.init_node(NODE_NAME)
    rospy.loginfo('Starting Node: ' + NODE_NAME)
    controller = ExperimentController()
    rospy.loginfo('Keyboard Manager node running')
    controller.stdscr = curses.initscr()

    # input_message = stdscr.getkey();
    while not rospy.is_shutdown():
        input_message = controller.stdscr.getkey()
        # input_message = str(raw_input("Options: \n Rotate robot to LEFT press:  a \n Rotate robot to RIGHT press: b \n"))
        controller.handle_input(input_message)


if __name__ == "__main__":
    main()
