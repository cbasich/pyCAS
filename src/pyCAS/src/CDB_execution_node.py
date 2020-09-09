#!/usr/bin/env python
import json
import os
import rospy
import roslib
import actionlib
from geometry_msgs.msg import Point, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from pyCAS.msg import RobotStatus, DoorStatus, SSPState, RobotAction, RobotGoal

CURRENT_FILE_PATH = os.path.dirname(os.path.realpath(__file__))

##############
# Publishers #
##############

# TODO: add the RobotAction message - or change this to be an interaction message??
#SSP_ACTION_PUBLISHER = rospy.Publisher("robot/robot_action", RobotAction.msg, queue_size=1)
# this is necessary to know the base of the robot and to set the new location for the robot to move 
# NAVIGATION_SERVICE.send_goal(next_location)
NAVIGATION_SERVICE = actionlib.SimpleActionClient('move_base', MoveBaseAction)

#############
# CALLBACKS #
#############

ssp_state_message = None

def ssp_state_callback(message):
    global ssp_state_message
    ssp_state_message = message
    state = task_handler.get_state_from_message(ssp_state_message)
    action = task_handler.get_action(state)
    if action is None:
        return
    ssp_action_message = SSPAction()
    # TODO: populate action message
    SSP_ACTION_PUBLISHER.publish(ssp_action_message)


def execute(message):
    wait_duration = rospy.get_param('/CDB_execution_node/wait_duration')
    timeout_duration = rospy.get_param('/CDB_execution_node/timeout_duration')

    action = message.action
    level = message.level

    # TODO: Populate action logic here.
    #       Should have logic for each of 4 LoA


def instantiate(message):
    world_map = json.load()
    end = message.goal
    start = task_handler.get_state_from_message(ssp_state_message)

    rospy.loginfo("Info[CDB_execution_node.instantiate]: Instantiating the domain model...")
    DM = CDB_domain_model.DeliveryBotDomain(world_map, start, end)

    rospy.loginfo("Info[CDB_execution_node.instantiate]: Instantiating the autonomy model...")
    AM = CDB_autonomy_model.AutonomyModel(DM, [0,1,2,3,])

    rospy.loginfo("Info[CDB_execution_node.instantiate]: Instantiating the feedback model...")
    HM = CDB_feedback_model.FeedbackModel(DM, AM, ['+', '-', '/', None], ['open'])

    rospy.loginfo("Info[CDB_execution_node.instantiate]: Instanaitating the CAS model...")
    CAS = competence_aware_system.CAS(DM, AM, HM, persistence = 0.75)

    CAS.solve()

def main():
    rospy.loginfo("Info[CDB_execution_node.main]: Instantiating the CDB_execution node...")
    rospy.init_node("CDB_execution_node", anonymous=True)

    rospy.Subscriber("monitor/ssp_state_monitor", SSPState, ssp_state_callback, queue_size=1)
    rospy.Subscriber("robot/robot_action", RobotAction, execute, queue_size=1)
    rospy.Subscriber("robot/robot_goal", RobotGoal, instantiate, queue_size=1)

    NAVIGATION_SERVICE.wait_for_server()

    rospy.loginfo("Info[CDB_execution_node.main]: Spinning...")
    rospy.spin()


if __name__ == '__main__':
    main()
