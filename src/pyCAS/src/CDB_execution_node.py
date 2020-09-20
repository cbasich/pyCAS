#!/usr/bin/env python3
import json
import os
import numpy as np
import rospy
import roslib
import actionlib

# ROS messages
from geometry_msgs.msg import Point, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# pyCAS specific
from pyCAS.msg import RobotStatus, ObstacleStatus, SSPState, TaskRequest, Interaction
from task_handler import CASTaskHandler
from models.CDB_robot.competence_aware_system import CAS

### GLOBALS ###

# navigation service - used to move the robot from one location to the next location
NAVIGATION_SERVICE = actionlib.SimpleActionClient("move_base", MoveBaseAction)

# publisher that sends the consequence of the human response depdending on LoA
# ex: if LoA is 1, the robot needs permission, if human says yes to proceeding in opening a door, the interaction will send a message "open"
INTERACTION_PUBLISHER = rospy.Publisher("monitor/interaction", Interaction, queue_size=10)


SSP_STATE_MESSAGE = None

def ssp_state_callback(message):
    global SSP_STATE_MESSAGE
    SSP_STATE_MESSAGE = message


def update_interaction_with_open():
    """
    params:
        None

    returns:
        Publishes an interaction message of door status = "open" 
        Only publishes a message when the door status is open 
    """
    interaction = Interaction()
    interaction.status = "open"
    INTERACTION_PUBLISHER.publish(interaction)


def get_human_interaction(current_state, current_action, model, is_sim):
    """
    params:
        Current state: ((row, col, direction, door status), LoA)
        Current action: ((+/- increment row, +/- increment col), LoA)
        Model: current CAS model 
        Simulation flag: boolean to define if this is a simulation run 

    returns:
        Flag to detemine if a new policy and state map needs to be generated
        Model that might be updated with a new transition if an action is denied 
    """
    generate_new_policy_flag = 0
    # LoA 0: Human does the action
    if current_action[1] == 0:
        rospy.loginfo("Level 0 Autonomy: Will wait for human interference to remove obstacle... ")

        # gets verification of obstacle removed
        # bypassing the user input if simulation
        if is_sim:
            response = "y"
        else:
            response = input("Have you removed the obstacle? [y/n]: ")

        # check response to see if the door has been opened
        if response[0] == "y" or response[0] == "Y":
            # update the interaction status with door "open"
            update_interaction_with_open()
        else:
            rospy.loginfo("[ERROR]: Invalid input...")
            #TODO: figure out what we want to do if invalid input 

    # LoA 1: Robot requests permission
    elif current_action[1] == 1:
        rospy.loginfo("Level 1 Autonomy: Ask permission to remove obstacle... ")

        # check door status to determine response for simulation
        if is_sim:
            doortype = SSP_STATE_MESSAGE.obstacle_status.obstacle_data
            if doortype == "push":
                response = "y"
            elif doortype == "pull":
                response = "n"
            else:
                rospy.loginfo("[ERROR] did not find doortype: {}\n".format(doortype))
        else:
            response = input("May I open the door? [y/n]: ")

        # check response to either open the door or have to generate new transition function
        if response[0] == "y" or response[0] == "Y":
            # verify the obstacle has been moved if real-world experiment 
            if not message.is_sim:
                input('Press any key to confirm that the door has been opened: ')
            
            update_interaction_with_open()

        # have to remove option from transition and resolve 
        elif response[0] == "n" or response[0] == "N":
            rospy.loginfo("Removing action from transition function... ")
            model.remove_transition(current_state, current_action)
            generate_new_policy_flag = 1
            # restart to current state of null
            current_state = None

        else:
            rospy.loginfo("[ERROR]: Invalid input...")

    # LoA 2: Robot acts under supervision
    elif current_action[1] == 2:
        # simulating response 
        if is_sim:
            response = "y"
        else:
            response = input('Are you supervising my actions?')
        
        # checking response to publish interaction 
        if response[0] == "y" or response[0] == "Y":
            update_interaction_with_open()
        
        # if response is no, then the action will be removed from the transition 
        elif response[0] == "n" or response[0] == "N":
            rospy.loginfo("Removing action from transition function... ")
            model.remove_transition(current_state, current_action)
            generate_new_policy_flag = 1 
            # restart to current state of null
            current_state = None
    return generate_new_policy_flag, model 


def go_back_to_start(start):
    """
    params:
        Start position (row, col)

    returns:
        Publishes the start position as the next position through move base planner 
    """
    # after the goal has been completed, send robot back to the start to repeat 
    target_pose = task_handler.get_pose(start)
    x = target_pose[0]
    y = target_pose[1]

    # populate the move base planner goal with the start location info 
    next_location = MoveBaseGoal()
    next_location.target_pose.header.frame_id = "map"
    next_location.target_pose.header.stamp = rospy.Time.now()
    next_location.target_pose.pose = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1))

    print("Returning to start")
    print(next_location)
    NAVIGATION_SERVICE.send_goal(next_location)

    status = NAVIGATION_SERVICE.wait_for_result()
    if not status:
        raise RuntimeError("Failed to reach the action server")
    

### EXECUTE ###

def execute(message):
    """
    params:
        Taske request message that contains the goal and a boolean to flag when to use simulation mode 

    returns:
        Executes the CAS robot code  
    """
    # setup parameters
    start_x = None
    start_y = None
    current_state = None

    # load parameters from launch file
    wait_duration = rospy.get_param("/CDB_execution_node/wait_duration")
    timeout_duration = rospy.get_param("/CDB_execution_node/timeout_duration")
    obstacle_map = rospy.get_param("/CDB_execution_node/obstacle_map")
    grid_map = rospy.get_param("/CDB_execution_node/grid_map")
    
    #  simulates multiple iterations but only does one iteration for real world experiments  
    if message.is_sim:
        number_iterations = int(rospy.get_param("/CDB_execution_node/number_iterations"))
    else:
        number_iterations = 1

    # loading the obstacle waypoints
    obstacle_map = json.load(open(obstacle_map))

    # this handles helper functions related to the task request
    task_handler = CASTaskHandler()

    # delaying the code to get the start position from odom data
    while not SSP_STATE_MESSAGE:
        rospy.loginfo(
            "Info[CDB_execution_node.execute]: waiting to get start position..."
        )
        rospy.sleep(wait_duration)

    start_row = SSP_STATE_MESSAGE.robot_status.location_row
    start_col = SSP_STATE_MESSAGE.robot_status.location_col

    # start position (row, col)
    start = (start_row, start_col)
    # an array of goals 
    goals = task_handler.parse_goals(message.goals)

    # will iterate the according to simulation or real life
    for ii in range(number_iterations):
        rospy.loginfo("\n\nThis is iteration {}\n\n".format(ii))

        # choosing a random goal from task request message
        goal = goals[np.random.choice(len(goals))]
        rospy.loginfo(
            "Info[CDB_execution_node.execute]: The goal has been selected : {}".format(
                goal
            )
        )

        # getting problem and solution for this iteration
        model = task_handler.get_problem(grid_map, start, goal)
        policy, state_map = task_handler.get_solution(model)
        rospy.loginfo("Info[CDB_execution_node.instantiate]: Retrieved solution...")

        # checks the percent of the correct actions
        level = model.check_level_optimality()
        rospy.loginfo("\n\nLevel of optimality is {}\n\n".format(level))

        # loop until the task is completed 
        while not task_handler.is_goal(current_state, goal):
            new_state = task_handler.get_state(SSP_STATE_MESSAGE)
            print(new_state)

            if new_state != current_state:
                # get the optimal action for the current state
                current_state = new_state
                state_index = state_map[current_state]
                current_action = policy[state_index]

                response = None
                if current_action[1] != 3:
                    generate_new_policy_flag, model = get_human_interaction(current_state, current_action, model, message.is_sim)
                    if generate_new_policy_flag:
                        policy, state_map = task_handler.get_solution(model)
                
                # LoA is 3 which means that it can act fully autonomously 
                else:
                    # determine the next location based off of current action and current state
                    target_state = (
                        current_state[0][0] + current_action[0][0],
                        current_state[0][1] + current_action[0][1],
                    )
                    target_pose = task_handler.get_pose(target_state)
                    x = target_pose[0]
                    y = target_pose[1]

                    # publish the next location to the move base planner 
                    next_location = MoveBaseGoal()
                    next_location.target_pose.header.frame_id = "map"
                    next_location.target_pose.header.stamp = rospy.Time.now()
                    next_location.target_pose.pose = Pose(
                        Point(x, y, 0), Quaternion(0, 0, 0, 1)
                    )
                    # verify that the correct location is being published
                    print(next_location)

                    NAVIGATION_SERVICE.send_goal(next_location)

                    status = NAVIGATION_SERVICE.wait_for_result()
                    if not status:
                        raise RuntimeError("Failed to reach the action server")

            rospy.sleep(wait_duration)

        rospy.loginfo("Info[CDB_execution_node.execute]: Completed the task")
        
        # after completeing the task, send the robot back to the start position
        go_back_to_start(start)
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


if __name__ == "__main__":
    main()


# TODO: object detection with QR codes
