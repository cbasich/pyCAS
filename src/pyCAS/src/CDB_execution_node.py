#!/usr/bin/env python
import json
import os
import rospy
import roslib
import actionlib
from geometry_msgs.msg import Point, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal'

from pyCAS.msg import RobotStatus, DoorStatus, SSPState, Interaction, RobotGoal
from task_handler import CASTaskHandler

CURRENT_FILE_PATH = os.path.dirname(os.path.realpath(__file__))

##############
# Publishers #
##############

# This will be the topic that will command the robot once it hits an area that it needs human interaction  
SSP_INTERACTION_PUBLISHER = rospy.Publisher("robot/robot_interaction", RobotAction.msg, queue_size=1)
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
    # state = (message.RobotStatus.x_coord, message.RobotStatus.y_coord) 
    # action = task_handler.get_action(state)
    # if action is None:
    #     return
    # ssp_action_message = SSPAction()
    # # TODO: populate action message
    # SSP_ACTION_PUBLISHER.publish(ssp_action_message)

def execute(message):
    wait_duration = rospy.get_param('/CDB_execution_node/wait_duration')
    timeout_duration = rospy.get_param('/CDB_execution_node/timeout_duration')
    obstacle_map = rospy.get_param('/CDB_execution_node/obstacle_map')
    task_handler = CASTaskHandler()
    # this has the waypoints that have obstacles  
    world_map = json.load(open(topological_map))
    # start position from odom data
    start_x = ssp_state_message.robot_status.x_coord
    start_y = ssp_state_message.robot_status.y_coord

    goal = (message.goal_x, message.goal_y)

    policy = task_handler.get_solution(world_map, (start_x, start_y), goal)
    
    current_state = None
    
    while not task_handler.is_goal(current_state, task_data):
        # state is in format ((x, y, theta), obstacle_status)
        new_state = task_handler.get_state(ssp_state_message)
        if new_state != current_state:
            current_state = new_state
            current_action = policy[current_state]
            # something along this line to get the next state 
            target_state = current_state + current_action
            
            # if there is an obstacle there will be an obstacle type published. if not, None will be published
            if ssp_state_message.obstacle_status.obstacle_type: 
                # TODO human interaction handling 
                pass
            else: 
                x = target_state[0]
                y = target_state[1]
                next_location = MoveBaseGoal()
                next_location.target_pose.header.frame_id = 'map'
                next_location.target_pose.header.stamp = rospy.Time.now()
                next_location.target_pose.pose = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1))

                NAVIGATION_SERVICE.send_goal(next_location)

                status = NAVIGATION_SERVICE.wait_for_result()
                if not status:
                    raise RuntimeError('Failed to reach the action server')
            
        rospy.sleep(wait_duration)
    rospy.loginfo('Info[CDB_execution_node.execute]: Completed the task')

    # TODO get start point from location monitor 


    # action = message.action
    # level = message.level

    # TODO: Populate action logic here.
    #       Should have logic for each of 4 LoA
    # TODO: Create if statement logic for when to puiblish an interaction command - this should be points where the robot stops to ask the human




""" def instantiate(message):
    global goal_message
    world_map = json.load()
    end = message.goal
    goal_message = end
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
    policy = CAS.pi """

def main():
    rospy.loginfo("Info[CDB_execution_node.main]: Instantiating the CDB_execution node...")
    rospy.init_node("CDB_execution_node", anonymous=True)

    # this will contain the monitor of the current robot location and obstacle detection
    rospy.Subscriber("monitor/ssp_state_monitor", SSPState, ssp_state_callback, queue_size=1)
    # this will be the topic to tell the robot the goal and any other task related items 
    rospy.Subscriber("robot/task_request", TaskRequest, execute, queue_size=1)


    NAVIGATION_SERVICE.wait_for_server()

    rospy.loginfo("Info[CDB_execution_node.main]: Spinning...")
    rospy.spin()


if __name__ == '__main__':
    main()
