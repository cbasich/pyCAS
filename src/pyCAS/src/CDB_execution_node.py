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
# NAVIGATION_SERVICE.send_goal(next_location)
NAVIGATION_SERVICE = actionlib.SimpleActionClient('move_base', MoveBaseAction)

# this is the publish that sends the consequence of the human response depdending on LoA 
# ex: if LoA is 1, the robot needs permission, if human says yes to proceeding in opening a door, the interaction will send a message "open"
INTERACTION_PUBLISHER = rospy.Publisher("monitor/interaction", Interaction, queue_size=10)

#############
# CALLBACKS #
#############

ssp_state_message = None
interaction_message = None

def ssp_state_callback(message):
    global ssp_state_message
    ssp_state_message = message


def interaction_callback(message):
    global interaction_message
    interaction_message = message

###########
# EXECUTE #
###########

def execute(message):
    # setup parameters 
    start_x = None
    start_y = None
    wait_duration = rospy.get_param('/CDB_execution_node/wait_duration')
    timeout_duration = rospy.get_param('/CDB_execution_node/timeout_duration')
    obstacle_map = rospy.get_param('/CDB_execution_node/obstacle_map')
    grid_map = rospy.get_param('/CDB_execution_node/grid_map')
    number_iterations = int(rospy.get_param('/CDB_execution_node/number_iterations'))
    # this has the waypoints that have obstacles  
    world_map = json.load(open(obstacle_map))
    # this handles the CAS model 
    task_handler = CASTaskHandler()
    
    # start position from odom data - wait for the init delay
    while not ssp_state_message:
        rospy.loginfo('Info[CDB_execution_node.execute]: waiting to get start position...')
        rospy.sleep(wait_duration)
   
    start_row = ssp_state_message.robot_status.location_row
    start_col = ssp_state_message.robot_status.location_col

    start = (start_row, start_col)
    goals = task_handler.parse_goals(message.goals)

    
    
    current_state = None
    for i in range(number_iterations):
        rospy.loginfo("\n\nThis is iteration {}\n\n".format(i))
        goal = goals[np.random.choice(len(goals))]
        rospy.loginfo("Info[CDB_execution_node.execute]: The goal has been selected : {}".format(goal))
        model = task_handler.get_problem(grid_map, start, goal)
        policy, state_map = task_handler.get_solution(model)
        rospy.loginfo("Info[CDB_execution_node.instantiate]: Retrieved solution...")
        # checks the percent of the correct 
        level = model.check_level_optimality()
        rospy.loginfo("\n\nLevel of optimality is {}\n\n".format(level))
        while not task_handler.is_goal(current_state, goal):
            # state is in format ((x, y, theta), obstacle_status)
            new_state = task_handler.get_state(ssp_state_message)

            if new_state != current_state:
                current_state = new_state
                state_index = state_map[current_state]
                current_action = policy[state_index]
  
                response = None
                if current_action[1] != 3: 
                    # LoA 0: Human does the action
                    if current_action[1] == 0:
                        rospy.loginfo("Level 0 Autonomy: Will wait for human interference to remove obstacle... ")
                        # FOR SIMS
                        # response = input("Have you removed the obstacle? [y/n]: ")
                        # TODO: create a if sim argument to be passed 
                        response = 'y'
                        if response[0] == 'y' or response[0] == 'Y':
                            interaction = Interaction()
                            interaction.status = 'open'
                            INTERACTION_PUBLISHER.publish(interaction)
                        else:
                            rospy.loginfo("Invalid input. Will continue to wait until the obstacle has been removed and the input is 'yes'.")
                        response = None
                    # LoA 1: Robot requests permission
                    elif current_action[1] == 1:
                        rospy.loginfo("Level 1 Autonomy: Ask permission to remove obstacle... ")
                        # FOR SIMS
                        # response = input("May I open the door? [y/n]: ")
                        doortype = ssp_state_message.obstacle_status.obstacle_data
                        if doortype == 'push':
                            response = 'y'
                        elif doortype == 'pull':
                            response = 'n'
                        else:
                            rospy.loginfo("ERROR in obstacle status - did not find doortype: {}\n".format(doortype))
                        if response[0] == 'y' or response[0] == 'Y':
                            # FOR SIMS
                            # input('Press any key to confirm that the door has been opened: ')
                            interaction = Interaction()
                            interaction.status = 'open'
                            INTERACTION_PUBLISHER.publish(interaction)
                        # request permission
                        elif response[0] == 'n' or response[0] == 'N':
                            rospy.loginfo('Removing action from transition function... ')
                            model.remove_transition(current_state, current_action)
                            policy, state_map = task_handler.get_solution(model)
                            current_state = None
                            
                        else:
                            rospy.loginfo("Invalid input")
                    # LoA 2: Robot acts under supervision
                    elif current_action[1] == 2:
                        # FOR SIMS
                        # response = input('Are you supervising my actions?')
                        rospy.loginfo('Level 2 Autonomy: Not sure what to do ?')
                        response = 'y'
                        if response[0] == 'y' or response[0] == 'Y':
                            interaction = Interaction()
                            interaction.status = 'open'
                            INTERACTION_PUBLISHER.publish(interaction) 
                        elif response[0] == 'n' or response[0] == 'N':
                            rospy.loginfo('Removing action from transition function... ')
                            model.remove_transition(current_state, current_action)
                            policy, state_map = task_handler.get_solution(model)
                            current_state = None  
                else: 
                    target_state = (current_state[0][0] + current_action[0][0], current_state[0][1] + current_action[0][1])
                    target_pose = task_handler.get_pose(target_state)
                    x = target_pose[0]
                    y = target_pose[1]
                    
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
        # send robot back to the start 
        target_pose = task_handler.get_pose(start)
        x = target_pose[0]
        y = target_pose[1]
        next_location = MoveBaseGoal()
        next_location.target_pose.header.frame_id = 'map'
        next_location.target_pose.header.stamp = rospy.Time.now()
        next_location.target_pose.pose = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1))

        NAVIGATION_SERVICE.send_goal(next_location)

        status = NAVIGATION_SERVICE.wait_for_result()
        if not status:
            raise RuntimeError('Failed to reach the action server')
        current_state = None


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


# TODO: clean up the states and action to either be all x, y or row, col 
# TODO: fix the paths so clean works before running roslauch 
# TODO: figure out what you want to do when you tell the robot that it cannot open a door
# TODO: check that the door status gets cleared when you go to the next state 
# TODO: object detection with QR codes