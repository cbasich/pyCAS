#!/usr/bin/env python3
import json
import os
import numpy as np
import rospy
import roslib
import actionlib
from geometry_msgs.msg import Point, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from pyCAS.msg import RobotStatus, ObstacleStatus, SSPState, TaskRequest, Interaction
from task_handler import CASTaskHandler
from models.CDB_robot.competence_aware_system import CAS

CURRENT_FILE_PATH = os.path.dirname(os.path.realpath(__file__))

##############
# Publishers #
##############

# this is necessary to know the base of the robot and to set the new location for the robot to move 
NAVIGATION_SERVICE = actionlib.SimpleActionClient('move_base', MoveBaseAction)

###########
# EXECUTE #
###########
X_GOAL = 4.02
Y_GOAL = -0.27
X_START = -0.328
Y_START = 0.136

def execute(message):
    # setup parameters 
    start_x = None
    start_y = None

    counter = 0
    while not rospy.is_shutdown():
        # publishing back and forth between goal and start to test the robot movement 
        if counter == 0:
            counter += 1
            x = X_GOAL
            y = Y_GOAL
            
            # populating target location message to be published 
            next_location = MoveBaseGoal()
            next_location.target_pose.header.frame_id = 'map'
            next_location.target_pose.header.stamp = rospy.Time.now()
            next_location.target_pose.pose = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1))
            # verify correct location is being published
            print(next_location)

            # send and wait for status of message 
            NAVIGATION_SERVICE.send_goal(next_location)

            status = NAVIGATION_SERVICE.wait_for_result()
            if not status:
                raise RuntimeError('Failed to reach the action server')

        if counter == 1:  
            counter = 0      
            x = X_START
            y = Y_START

            # populating target location message to be published
            next_location = MoveBaseGoal()
            next_location.target_pose.header.frame_id = 'map'
            next_location.target_pose.header.stamp = rospy.Time.now()
            next_location.target_pose.pose = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1))
            # verify correct location is being published
            print(next_location)

            # send and wait for status of message 
            NAVIGATION_SERVICE.send_goal(next_location)

            status = NAVIGATION_SERVICE.wait_for_result()
            if not status:
                raise RuntimeError('Failed to reach the action server')


def main():
    rospy.loginfo("Info[test_gazebo.main]: Instantiating the test_gazebo node...")
    rospy.init_node("test_gazebo", anonymous=True)
    # publish any message on robot/task_request to start code 
    rospy.Subscriber("robot/task_request", TaskRequest, execute, queue_size=1)
 
    NAVIGATION_SERVICE.wait_for_server()

    rospy.loginfo("Info[test_gazebo.main]: Spinning...")
    rospy.spin()


if __name__ == '__main__':
    main()
