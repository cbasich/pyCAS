#!/usr/bin/env python
import os
import sys
import rospy
import math
import json

# adding the current path to sys.path for relative importing
current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, ".."))
sys.path.append(current_file_path)

# importing the CAS specific models
from models.CDB_robot.domain_model import DeliveryBotDomain
from models.CDB_robot.autonomy_model import AutonomyModel
from models.CDB_robot.feedback_model import FeedbackModel
from models.CDB_robot.competence_aware_system import CAS

# importing utility functions 
from scripts.utils import init_open_data, init_full_open_data


# Globals
FEEDBACK_DATA_PATH = os.path.join(current_file_path, "..", "..", "domains", "CDB_robot", "feedback")


class CASTaskHandler(object):
    def __init__(self):

        # load the toplogical map that contains all of the state waypoints
        self.topological_map = json.load(open(rospy.get_param("/CDB_execution_node/topological_map")))


    def parse_goals(self, goal_string):
        """
        params:
            A list of goals from the task request message in a format as a string '[(str, str), (str, str)]' 

        returns:
            An array of goal tuples with each row/col as integers. 
            Final format will be [(goal1_row, goal1_col), (goal2_row, goal2_col)]
        """
        # parsing the goal message from task request - output ['row', 'col', 'row', 'col', ...etc]
        goal_array = (
            goal_string.strip("[")
            .replace("(", "")
            .replace(")", "")
            .strip("]")
            .replace(" ", "")
            .split(",")
        )
        goals = []
        goal = ()

        # Parsing the goal data from a flat array into an array of tuples [(goal1_row, goal1_col), (goal2_row, goal2_col)]
        counter = 0
        for goal_coord in goal_array:
            goal = goal + (int(goal_coord),)

            # checks to see if the goal has both row and col
            if counter == 1:
                # goal is now a tuple (row, col) and is added to the new goal array
                goals.append(goal)
                # reset goal to get the next tuple
                goal = ()
                # reset counter
                counter = 0

            else:
                counter += 1
        
        return goals


    def get_direction(self, yaw):
        """
        params:
            yaw (radians)

        returns:
            direction in format NORTH, SOUTH, EAST, or WEST
        """

        # convert yaw from radians to degrees
        if yaw < 0:
            angle_in_degrees = 360 + (yaw * (180 / math.pi))
        else:
            angle_in_degrees = yaw * (180 / math.pi)

        # convert yaw into state representation (NORTH, SOUTH, EAST, WEST)
        if angle_in_degrees < 45 and angle_in_degrees > 0:
            direction = "EAST"

        elif angle_in_degrees < 360 and angle_in_degrees > 315:
            direction = "EAST"

        elif angle_in_degrees > 45 and angle_in_degrees < 135:
            direction = "NORTH"

        elif angle_in_degrees > 135 and angle_in_degrees < 225:
            direction = "WEST"

        elif angle_in_degrees > 225 and angle_in_degrees < 315:
            direction = "SOUTH"
        else:
            # invalid yaw
            print("[ERROR invalid yaw value]: not able to find the direction with heading/yaw value of {}".format(yaw))

        return direction

    
    def get_state(self, message):
        """
        params:
            SSP State message - contains robot status and obstacle status 

        returns:
            Gets the state based off of the robot location and the obstacle status. 
            Format ((row, col) LoA) or ((row, col, direction, door status), LoA)
            LoA - level of autonomy 
        """

        # set the default values
        door_status = "door-closed"
        LoA = 3
        
        # checks to see if there is an obstacle present 
        if message.obstacle_status.obstacle_data != "None":   

            # get the direction of the robot 
            yaw = message.robot_status.heading
            direction = self.get_direction(yaw)

            # listen for interaction from human to see if the robot has been allowed to open the door
            # TODO: find a way to confirm that the door has been opened before changing door_status
            if message.obstacle_status.door_status == "open":
                door_status = "door-open"

            return ((message.robot_status.location_row, message.robot_status.location_col, direction, door_status), LoA)

        else:
            # no obstacle so just return state ((row, col), LoA)
            return ((message.robot_status.location_row, message.robot_status.location_col), LoA)


    def get_pose(self, state):
        """
        params:
            State in form ((row, col) LoA) or ((row, col, direction, door status), LoA). LoA means level of autonomy 

        returns:
            X and y coordinates of that particular state. These coordinates are stored in the topoligical map as waypoints. 
        """

        pose = self.topological_map["states"][str(state)]["position"]
        # coordinates are in the map reference frame
        state_x = pose["x"]
        state_y = pose["y"]

        return (state_x, state_y)


    def is_goal(self, state, goal):
        """
        params:
            State: ((row, col) LoA) or ((row, col, direction, door status), LoA) 
            Goal: (row, col)

        returns:
            Boolean to see if the current state is the goal by comparing row and col values 
        """
        
        if state:
            current_state = (state[0][0], state[0][1])
        else:
            current_state = None

        return current_state == goal


    def get_problem(self, world_map, start, goal):
        """
        params:
            World map: this is the path text file representation of the state space that is needed for the Domain Model
            Start: (row, col) 
            Goal: (row, col) or 'G' 

        returns:
            Instantiates the domain model (DM), autonomy model (AM), feedback model (FM). 
            Returns the CAS model with inputs of DM, AM, and FM  
        """
        # set up the data files from feedback
        if not os.path.exists(os.path.join(FEEDBACK_DATA_PATH, "open.data")):
            init_open_data()
        if not os.path.exists(os.path.join(FEEDBACK_DATA_PATH, "open_full.data")):
            init_full_open_data()

        # Initiate all of the models
        rospy.loginfo("Info[task_handler.get_problem]: Instantiating the domain model...")
        DM = DeliveryBotDomain(world_map, start, goal)

        rospy.loginfo("Info[task_handler.get_problem]: Instantiating the autonomy model...")
        AM = AutonomyModel(DM, [0, 1, 2, 3])

        rospy.loginfo("Info[task_handler.get_problem]: Instantiating the feedback model...")
        HM = FeedbackModel(DM, AM, ["+", "-", "/", None], ["open"])

        rospy.loginfo("Info[task_handler.get_problem]: Instanaitating the CAS model...")
        cas_model = CAS(DM, AM, HM, persistence=0.75)
        
        return cas_model


    def get_solution(self, model):
        """
        params:
            CAS model that is already instantiated with current start, goal, level of autonomy, and statespace 

        returns:
            An optimal policy and state map from value iteration. 
            To retrieve the action of interest, first index state into state map to get the proper index for the policy. 
            Then use that index to get the action from the policy.   
        """
        # CAS model that has already been initiated by get_problem
        rospy.loginfo("Info[task_handler.get_solution]: Instantiating solver... ")
        model.solve()
        state_map = model.state_map
        policy = model.pi

        return policy, state_map
