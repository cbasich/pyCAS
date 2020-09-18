#!/usr/bin/env python3
import os 
print(os.getcwd())
import sys
import rospy
import math
import json

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))
sys.path.append(current_file_path)

OUTPUT_PATH = os.path.join(current_file_path, '..', '..', 'output', 'CDB_robot')
FEEDBACK_DATA_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB_robot', 'feedback')
PARAM_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB_robot', 'params_robot')
MAP_PATH = os.path.join(current_file_path, '..', '..', 'domains', 'CDB_robot', 'maps')


from models.CDB_robot.domain_model import DeliveryBotDomain
from models.CDB_robot.autonomy_model import AutonomyModel
from models.CDB_robot.feedback_model import FeedbackModel
from models.CDB_robot.competence_aware_system import CAS

from scripts.utils import init_cross_data, init_full_cross_data, init_open_data, init_full_open_data

class CASTaskHandler(object):
    def __init__(self):
        self.topological_map = json.load(open(rospy.get_param('/CDB_execution_node/topological_map')))

    def parse_goals(self, goal_string):
        # must be in format [(), (), ()]
        goal_array = goal_string.strip('[').replace('(', '').replace(')', '').strip(']').replace(' ', '').split(',')
        goals = []
        goal = ()
        # to make sure that you only get x, y data
        cntr = 0
        for location in goal_array:
            goal = goal + (int(location),)
            if cntr == 1:
                goals.append(goal)
                goal = ()
                cntr = 0
            else:
                cntr += 1
        print(goals)    
        return goals

    
    def get_state(self, message):
        # have to offset the odom data from the origin: [-0.45, -1.9, 0.0]
        if message.obstacle_status.obstacle_data != 'None':
            # obstacle_dict = eval(message.obstacle_status.obstacle_data)
            if message.obstacle_status.obstacle_data == 'pull' or message.obstacle_status.obstacle_data == 'push':
                # set the default door status to closed 
                door_status = 'door-closed'
                yaw = message.robot_status.heading
                # convert yaw from radians to degrees
                if yaw < 0:
                    angle_in_degrees = 360 + (yaw * (180/math.pi))  
                else:
                    angle_in_degrees= yaw *(180/math.pi)
                # convert yaw into state representation (NORTH, SOUTH, EAST, WEST)
                if angle_in_degrees < 45 and angle_in_degrees > 0:
                    compass = 'EAST'
                elif angle_in_degrees < 360 and angle_in_degrees > 315:
                    compass = 'EAST'
                elif angle_in_degrees > 45 and angle_in_degrees < 135:
                    compass = 'NORTH'
                elif angle_in_degrees > 135 and angle_in_degrees < 225:
                    compass = 'WEST'
                elif angle_in_degrees > 225 and angle_in_degrees < 315 :
                    compass = 'SOUTH'
                else:
                    print("Error: not able to find compass with heading value of {}".format(yaw)) 
                
                # listen for interaction from human to see if the robot has been allowed to open the door
                # right now we do not confirm that it has been opened - we just assume that it has been
                # TODO: find a way to confirm that the door has been opened before changing door_status 
                if message.obstacle_status.door_status == 'open':
                    door_status = 'door-open'
                return ((message.robot_status.location_row, message.robot_status.location_col, compass ,door_status), 3)
        else:
            # default LoA is 3 which means unsupervised autonomy 
            return ((message.robot_status.location_row, message.robot_status.location_col), 3)
    
    def get_pose(self, state):
        pose = self.topological_map["states"][str(state)]["pose"]
        state_x = pose['x']
        state_y = pose['y']
        return (state_x, state_y)

    def is_start(self, message):
        start = (message.robot_status.y_coord, message.robot_status.x_coord)
        pass

    def is_goal(self, state, goal):
        if state:
            current_state = (state[0][0], state[0][1])
        else:
            current_state = None
        return current_state == goal

    def get_problem(self, world_map, start, goal):
        # set up the data files from feedback 
        if not os.path.exists( os.path.join(FEEDBACK_DATA_PATH, 'cross.data') ):
            init_cross_data()
        if not os.path.exists( os.path.join(FEEDBACK_DATA_PATH, 'open.data') ):
            init_open_data()
        if not os.path.exists( os.path.join(FEEDBACK_DATA_PATH, 'cross_full.data') ):
            init_full_cross_data()
        if not os.path.exists( os.path.join(FEEDBACK_DATA_PATH, 'open_full.data') ):
            init_full_open_data()

        # Initiate all of the models

        rospy.loginfo("Info[task_handler.get_problem]: Instantiating the domain model...")
        # changed DM to accept start and goal of '(x, y)' or a text like 'S' or 'G'
        DM = DeliveryBotDomain(world_map, start, goal)

        rospy.loginfo("Info[task_handler.get_problem]: Instantiating the autonomy model...")
        AM = AutonomyModel(DM, [0,1,2,3])

        rospy.loginfo("Info[task_handler.get_problem]: Instantiating the feedback model...")
        HM = FeedbackModel(DM, AM, ['+', '-', '/', None], ['open'])

        rospy.loginfo("Info[task_handler.get_problem]: Instanaitating the CAS model...")
        cas_model = CAS(DM, AM, HM, persistence = 0.75)
        # CAS model to be returned and passed into get_solution
        return cas_model



    def get_solution(self, model):
        # CAS model that has already been initiated by get_problem
        rospy.loginfo("Info[task_handler.get_solution]: Instantiating solver... ")
        model.solve()
        state_map = model.state_map
        policy = model.pi
        return policy, state_map

