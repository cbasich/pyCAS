#!/usr/bin/env python
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
from scripts.utils import init_open_data, init_full_open_data

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


def update_interaction(door_status_boolean):
    """
    params:
        Door status boolean: false means door is closed and true means door is open

    returns:
        Publishes an interaction message of door status = "open" 
        Only publishes a message when the door status is open 
    """
    interaction = Interaction()

    if door_status_boolean:
        interaction.status = "open"
    else:
        interaction.status = "closed"

    INTERACTION_PUBLISHER.publish(interaction)


def get_human_interaction(model, current_state, current_action, is_sim):
    """
    params:
        Current action: ((+/- increment row, +/- increment col), LoA) or (('open'), LoA)
            Check to see what Level of Autonomy (LoA) to decide the need of human interaction. 
            Then, prompts the user if a response if necessary.
        Simulation flag: boolean to define if this is a simulation run 

    returns:
        Flag to detemine if a new policy and state map needs to be generated
    """
    generate_new_policy_flag = 0
    # LoA 0: Human does the action
    if current_action[1] == 0:
        rospy.loginfo("Level 0 Autonomy: Will wait for human interference to remove obstacle... ")

        # bypassing the user input if simulation
        if is_sim:
            response = "y"
        else:
            # gets verification of obstacle removed
            response = input("Have you removed the obstacle? [y/n]: ")

        # check response to see if the door has been opened
        if response[0] == "y" or response[0] == "Y":
            # update the interaction status with door "open"
            update_interaction(True)
        else:
            #TODO: figure out what we want to do if invalid input 
            rospy.loginfo("[ERROR]: Invalid input...")

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

        # check response to either open the door or to generate new transition function
        if response[0] == "y" or response[0] == "Y":
            
            # verify the obstacle has been moved if real-world experiment 
            if not is_sim:
                input('Press any key to confirm that the door has been opened: ')

            # Update the dataset. TODO: Move this to a "data/learning monitor"
            feedback = "yes"
            model.update_data(current_state, current_action, feedback)
            
            # assumes door has been opened and updates interaction message
            update_interaction(True)

        # sets flag to remove action from transition and generate a new policy
        elif response[0] == "n" or response[0] == "N":
            # make sure human interaction message has door status = "closed "
            update_interaction(False)
            generate_new_policy_flag = 1
            feedback = "no"
            model.update_data(current_state, current_action, feedback)

        else:
            rospy.loginfo("[ERROR]: Invalid input...")

    # LoA 2: Robot acts under supervision
    elif current_action[1] == 2:
        # simulating response 
        if is_sim:
            response = "y"
        else:
            # checks that the user is ready for the robot to continue
            response = input('Are you supervising my actions?')
        
        # checking response to publish interaction 
        if response[0] == "y" or response[0] == "Y":
            # assumes door has been opened and updates interaction message
            update_interaction(True)
            feedback = "yes"
            model.update_data(current_state, current_action, feedback)
        
        # if response is no, then the action will be removed from the transition 
        elif response[0] == "n" or response[0] == "N":
            generate_new_policy_flag = 1
            feedback = "no"
            model.update_data(current_state, current_action, feedback)
        
    return generate_new_policy_flag 


def go_back_to_start(start):
    """
    params:
        Start position (row, col)

    returns:
        None
        After the task has been completed, publishes the start position as the next position through move base planner 
    """
    task_handler = CASTaskHandler()
    # after the goal has been completed, send robot back to the start to repeat 
    target_pose = task_handler.get_pose(start)
    x = target_pose[0]
    y = target_pose[1]

    # populate the move base planner goal with the start location info 
    next_location = MoveBaseGoal()
    next_location.target_pose.header.frame_id = "map"
    next_location.target_pose.header.stamp = rospy.Time.now()
    next_location.target_pose.pose = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1))

    NAVIGATION_SERVICE.send_goal(next_location)

    status = NAVIGATION_SERVICE.wait_for_result()
    if not status:
        raise RuntimeError("Failed to reach the action server")


    ### EXECUTE ###

def execute(message):
    """
    params:
        Task request message that contains the goal and a boolean to flag when to use simulation mode 

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

    # set up the data files from feedback
    if not os.path.exists(os.path.join("..", "domains", "CDB_robot", "feedback", "open.data")):
        init_open_data()
    if not os.path.exists(os.path.join("..", "domains", "CDB_robot", "feedback", "open_full.data")):
        init_full_open_data()
    
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
        rospy.loginfo("Info[CDB_execution_node.execute]: waiting to get start position...")
        rospy.sleep(wait_duration)

    start_row = SSP_STATE_MESSAGE.robot_status.location_row
    start_col = SSP_STATE_MESSAGE.robot_status.location_col

    # start position (row, col)
    start = (start_row, start_col)
    # an array of goals 
    goals = task_handler.parse_goals(message.goals)

    # will iterate to learn 
    for ii in range(number_iterations):
        rospy.loginfo("\n\nThis is iteration {}\n\n".format(ii))

        # choosing a random goal from task request message
        goal = goals[np.random.choice(len(goals))]
        rospy.loginfo("Info[CDB_execution_node.execute]: The goal has been selected : {}".format(goal))

        # getting problem and solution for this iteration
        model = task_handler.get_problem(grid_map, start, goal)
        policy, state_map = task_handler.get_solution(model)
        rospy.loginfo("Info[CDB_execution_node.instantiate]: Retrieved solution...")

        # checks the percent of the correct actions
        level = model.check_level_optimality()
        rospy.loginfo("\n\nLevel of optimality is {}\n\n".format(level))

        # initialize current state as None
        current_state = None

        # loop until the task is completed 
        while not task_handler.is_goal(current_state, goal):
            # get the most recent state
            new_state = task_handler.get_state(SSP_STATE_MESSAGE)

            if new_state != current_state:
                # get the optimal action for the current state
                current_state = new_state
                state_index = state_map[current_state]
                current_action = policy[state_index]

                # if the level of autonomy needs some human interaction (any level other than unsupervised = 3)
                if current_action[1] != 3:
                    # set default door status interaction to "closed " 
                    update_interaction(False)
                    # detemines the level of human interaction needed and retrives user input 
                    generate_new_policy_flag = get_human_interaction(model, current_state, current_action, message.is_sim)
                    # if the action has been denied, then it needs to be removed from the transition function 
                    if generate_new_policy_flag:
                        rospy.loginfo("Removing action from transition function... ")
                        model.remove_transition(current_state, current_action)
                        policy, state_map = task_handler.get_solution(model)
                        # reset the current state to get a fresh new state
                        current_state = None
                
                # LoA is 3 which means that it can act fully autonomously 
                else:
                    # determine the next location based off of current action and current state
                    target_state = (
                        current_state[0][0] + current_action[0][0],
                        current_state[0][1] + current_action[0][1]
                    )


                    # TODO: Move this function to the top
                    # TODO: Eliminate reliance on SSP_STATE_MESSAGE
                    def get_quaternion(current_location, successor_location):
                        import math
                        from tf.transformations import quaternion_from_euler

                        x_displacement = successor_location[0] - current_location[0]
                        y_displacement = successor_location[1] - current_location[1]
                        target_yaw = math.atan2(x_displacement, y_displacement)

                        yaw_displacement = target_yaw - SSP_STATE_MESSAGE.robot_status.yaw

                        return quaternion_from_euler(0, 0, yaw_displacement)
 
                    # get the target state position in reference to the map frame
                    target_pose = task_handler.get_pose(target_state)
                    x = target_pose[0]
                    y = target_pose[1]

                    # publish the next location to the move base planner 
                    next_location = MoveBaseGoal()
                    next_location.target_pose.header.frame_id = "map"
                    next_location.target_pose.header.stamp = rospy.Time.now()
                    next_location.target_pose.pose = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1))
                    # next_location.target_pose.pose = Pose(Point(x, y, 0), get_quaternion(current_state[0], target_state))

                    NAVIGATION_SERVICE.send_goal(next_location)

                    status = NAVIGATION_SERVICE.wait_for_result()
                    # checks that the goal was sent out properly 
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
