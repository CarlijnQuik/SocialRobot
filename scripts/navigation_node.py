#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Bool, Int16
import random
import math

NODE_NAME = 'navigation_node'
ROTATION_SPEED = 10
PI = math.pi
ANGULAR_SPEED = 0.78


class NavigationManager(object):


    def __init__(self):
        rospy.loginfo('[Experiment Controller] Initializing communication with managers and robot drivers.')
        self.angle_rotation_sub = rospy.Subscriber('movement_angle', Int16, self.handle_rotation)
        self.changing_robot_angle_pub = rospy.Publisher('joy', Twist, queue_size=10000)
        self.is_moving_sub = rospy.Subscriber('movement_state', Bool, self.handle_movement_state)

    def handle_rotation(self, angleData):
        angle = angleData.data
        rotationSign = angle / abs(int(angle))
        relative_angle = abs(angle) * 2 * PI/360

        vel_msg = self.getTwist()
        vel_msg.angular.z = rotationSign * ANGULAR_SPEED

        current_angle = 0
        t0 = rospy.Time.now().to_sec()

        self.odometry_sub = rospy.Subscriber('/odom', Odometry, self.handle_odometry)
        while (current_angle < relative_angle):
            self.changing_robot_angle_pub.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = ANGULAR_SPEED * (t1-t0)

        vel_msg.angular.z = 0
        self.changing_robot_angle_pub.publish(vel_msg)

    def handle_odometry(self, message):
        odometry_data = message

    def handle_movement_state(self, state):
        isMoving = state.data
        vel_msg = self.getTwist()
        if (isMoving):
            vel_msg.linear.x = 0.2
        else:
            vel_msg.linear.x = 0
        self.changing_robot_angle_pub.publish(vel_msg)

    def getTwist(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        return vel_msg

def main():
    rospy.init_node(NODE_NAME)
    rospy.loginfo('Starting Node: ' + NODE_NAME)
    navigationManger = NavigationManager()
    rospy.loginfo('[Navigation Manager] Node is working.')
    rate = rospy.Rate(5)
    rospy.spin()


if __name__ == "__main__":
    main()
