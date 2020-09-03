#!/usr/bin/env python
import json
import os
import rospy
import roslib

from msg import RobotStatus, DoorStatus, SSPState

CURRENT_FILE_PATH = os.path.dirname(os.path.realpath(__file__))

##############
# Publishers #
##############

NAVIGATION_ACTION_PUBLISHER = rospy.Publisher("orb_nav/goal", NavGoal, queue_size=1)
INTERFACE_ACTION_PUBLISHER = rospy.Publisher("task_execution/interface_action", InterfaceAction, queue_size=1)
ROBOT_STATUS_PUBLISHER = rospy.Publisher("monitor/robot_status", RobotStatus, queue_size=1)
TASK_STATUS_PUBLISHER = rospy.Publisher("monitor/task_status", TaskStatus, queue_size=1)

#############
# CALLBACKS #
#############

ssp_state_message = None

def ssp_state_callback(message):
    global ssp_state_message
    delivery_mdp_state_message = message


def execute():
    pass


def main():
    rospy.loginfo("Info[CDB_execution_node.main]: Instantiating the CDB_execution node...")
    rospy.init_node("CDB_execution_node", anonymous=True)

    rospy.Subscriber("monitor/ssp_state_monitor", SSPState, ssp_state_callback, queue_size=1)

    rospy.loginfo("Info[CDB_execution_node.main]: Spinning...")
    rospy.spin()


if __name__ == '__main__':
    main()
