#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import random
from inside_movement.msg import *
from inside_animation.srv import *
from std_msgs.msg import Bool
from navigation_utils.navigating_goals import TargetNavigation
from inside_animations.robot_gaze import GazeStates


NODE_NAME = 'inside_movement_manager'


class InsideMovementManager(object):
    def __init__(self):

        rospy.loginfo('[INSIDE Movement Manager] Initializing communication with robot.')

        # Movement variables
        self.current_location = ''
        self.last_location = ''
        self.current_behavior = 'static'
        self.last_behavior = ''
        self.finished_changing_movement = False
        self.waypoints = rospy.get_param('/navigation_goals')
        self.completed_movement = False

        # Movement client
        self.nav_client = TargetNavigation()
        self.nav_client.load_loci()

        # Face Services and Topics
        self.animation_change_movement_sub = rospy.Subscriber('inside_movement/movement', ChangeMovement, self.handle_change_movement)
        self.animation_change_movement_point_sub = rospy.Subscriber('inside_movement/point_movement',
                                                                    ChangeMovementPoint, self.handle_change_movement_point)
        self.moving_robot_pub = rospy.Publisher('inside_navigation/moving_robot', Bool, queue_size=10000)
        self.finished_clean_movement_pub = rospy.Publisher('inside_navigation/finished_clean_movement', Bool, queue_size=10000)

        # Establish animation movement service proxies
        connection_done = False
        first_try = rospy.get_time()
        while not connection_done and ((rospy.get_time() - first_try) < 60):
            try:
                rospy.wait_for_service('inside_animations/gaze_state_control', timeout=5)
                self.gaze_control_srv = rospy.ServiceProxy('inside_animations/gaze_state_control', SwitchGaze)
                connection_done = True
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr("Error getting service connections: %s" % e)

    def handle_change_movement(self, msg):

        # Start Changing Movement
        self.finished_changing_movement = False
        self.completed_movement = False

        # Save last values
        self.last_location = self.current_location
        self.last_behavior = self.current_behavior

        # If non empty location:
        if msg.location != '':
            # Save new value
            self.current_location = msg.location.lower()

            # Send to navigation new position
            print('[INSIDE MOVEMENT] Sending robot to new position = ' + self.current_location)
            self.moving_robot_pub.publish(True)
            print('[INSIDE MOVEMENT] Robot moving')
            self.completed_movement = self.nav_client.move_goal(self.waypoints[self.current_location])
            self.moving_robot_pub.publish(False)
            print('[INSIDE MOVEMENT] Completed movement clean = ' + str(self.completed_movement))
            self.finished_clean_movement_pub.publish(self.completed_movement)
            print('[INSIDE MOVEMENT] Robot not moving')
            self.current_behavior = 'static'

        else:
            # If movement type is cradle
            if msg.behavior == 'cradle':
                r1 = self.nav_client.turn_robot(90, 1.5)
                r2 = self.nav_client.turn_robot(-60, 1.5)
                r3 = self.nav_client.turn_robot(35, 1.5)

            elif msg.behavior == 'spin':
                spin_gaze_state = SwitchGazeRequest()
                spin_gaze_state.gaze_state = GazeStates.GO_NEUTRAL
                spin_gaze_state.velocity = 45
                self.gaze_control_srv(spin_gaze_state)

                if random.random() < 0.5:
                    print('DEU 1')
                    r1 = self.nav_client.turn_robot(180, 12.0)
                    r2 = self.nav_client.turn_robot(180, 12.0)
                else:
                    print('DEU 2')
                    r1 = self.nav_client.turn_robot(-180, 12.0)
                    r2 = self.nav_client.turn_robot(-180, 12.0)

                spin_gaze_state = SwitchGazeRequest()
                spin_gaze_state.gaze_state = GazeStates.FOLLOW_CHILD
                spin_gaze_state.velocity = 45
                self.gaze_control_srv(spin_gaze_state)

        #Finished changing movement
        self.finished_changing_movement = True
        print('[INSIDE Movement] Finished changing movement')

    def handle_change_movement_point(self, msg):
        # Start Changing Movement
        self.finished_changing_movement = False

        # Save last values
        self.last_location = self.current_location
        self.last_behavior = self.current_behavior

        # Save new value
        self.current_location = 'Point'
        # Send to navigation new position
        self.moving_robot_pub.publish(True)
        print('[INSIDE MOVEMENT] Robot moving')
        self.nav_client.move_goal(msg.data)
        self.moving_robot_pub.publish(False)
        print('[INSIDE MOVEMENT] Completed movement clean = ' + str(self.completed_movement))
        self.finished_clean_movement_pub.publish(self.completed_movement)
        print('[INSIDE MOVEMENT] Robot not moving')
        self.current_behavior = 'static'

        # Finished changing movement
        self.finished_changing_movement = True
        print('[INSIDE Movement] Finished changing movement')


def main():
    rospy.init_node(NODE_NAME)
    rospy.loginfo('Starting Node: ' + NODE_NAME)
    imm = InsideMovementManager()
    rospy.loginfo('Inside Movement Manager node running')
    rospy.spin()


if __name__ == "__main__":
    main()
