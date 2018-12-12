#!/usr/bin/env python
# -*- coding: utf-8 -*-


import random
import rospy
from std_msgs.msg import Bool, UInt32
from inside_movement.msg import *
from inside_animation.srv import *
from inside_animations.robot_gaze import GazeStates




NODE_NAME = 'inside_cmu_movement'


class InsideCmuMovement(object):
    def __init__(self):

        rospy.loginfo('[INSIDE CMU Movement] Initializing communication with robot.')

        # Variables
        self.cmu_waypoints = rospy.get_param('/cmu_review/waypoints/')
        self.should_move = False
        self.is_moving = False
        self.obstacle = False
        self.first_stop_msg = True
        self.shutdown = False
        self.last_points = []

        # Timer
        self.timer_obstacle_in_the_way = rospy.Duration(6.0)
        self.obstacle_in_the_way_stime = rospy.Time.now()

        # CMU Movement Topics
        self.cmu_movement = rospy.Subscriber('inside_cmu_review/movement', Bool, self.handle_should_move_update)
        self.cmu_obstacle_det = rospy.Subscriber('inside_cmu_review/obstacle', Bool, self.handle_obstacle_update)
        self.moving_robot_pub = rospy.Subscriber('inside_navigation/moving_robot', Bool, self.handle_is_moving_update)
        self.animation_change_movement_point_pub = rospy.Publisher('inside_movement/point_movement', ChangeMovementPoint, queue_size=100000)
        self.stop_moving = rospy.Publisher('navigation/stop_moving', Bool, queue_size=100000)

        # CMU Movement services
        # Services
        try_connection = True
        while try_connection:
            try:

                rospy.wait_for_service('inside_animations/gaze_state_control', timeout=5)
                self.gaze_control_srv = rospy.ServiceProxy('inside_animations/gaze_state_control', SwitchGaze)

                try_connection = False

            except (rospy.ServiceException, rospy.ROSException) as e:
                print("Service call change_face failed: %s" % e)

    def handle_obstacle_update(self, msg):
        self.obstacle = msg.data

    def handle_should_move_update(self, msg):
        self.should_move = msg.data
        if not self.should_move and self.first_stop_msg:
            self.stop_moving.publish(True)
            self.first_stop_msg = False
        if self.should_move:
            self.first_stop_msg = True

    def handle_is_moving_update(self, msg):
        self.is_moving = msg.data
        if not msg.data:
            # Gaze: Neutral
            motivate_gaze_state = SwitchGazeRequest()
            motivate_gaze_state.gaze_state = GazeStates.GO_NEUTRAL
            motivate_gaze_state.velocity = 30
            self.gaze_control_srv(motivate_gaze_state)
        else:
            # Gaze: Neutral
            motivate_gaze_state = SwitchGazeRequest()
            motivate_gaze_state.gaze_state = GazeStates.IDLE_BASIC
            motivate_gaze_state.velocity = 30
            self.gaze_control_srv(motivate_gaze_state)

    def update_behavior(self):

        if not self.is_moving and self.should_move:
            print('[INSIDE CMU MOVEMENT] Sending a waypoint')
            point = random.choice(self.cmu_waypoints)
            print('Point: ' + str(point))
            print(self.last_points)
            print(point in self.last_points)
            if point not in self.last_points:
                print('[INSIDE CMU MOVEMENT] X = ' + str(point[0]) + ' | Y = ' + str(point[1]))
                self.animation_change_movement_point_pub.publish(point)
                if len(self.last_points) >= 2:
                    self.last_points.pop(0)
                    self.last_points += [point]
                rospy.sleep(1)
            else:
                print('[INSIDE CMU MOVEMENT] Chose same point')

    def shutdown_cmu_movement(self):
        self.shutdown = True


def main():
    rospy.init_node(NODE_NAME)
    rospy.loginfo('Starting Node: ' + NODE_NAME)
    icm = InsideCmuMovement()
    rospy.loginfo('[INSIDE CMU MOVEMENT] Node running')
    rate = rospy.Rate(5)
    while not icm.shutdown:
        icm.update_behavior()
        rate.sleep()


if __name__ == "__main__":
    main()
