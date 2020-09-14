#!/usr/bin/env python3
import os 
print(os.getcwd())
import sys
import rospy

current_file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_file_path, '..'))


from models.CDB.domain_model import DeliveryBotDomain
from models.CDB.autonomy_model import AutonomyModel
from models.CDB.feedback_model import FeedbackModel
from models.CDB.competence_aware_system import CAS

class CASTaskHandler(object):
    def get_state(self, message):
        # have to offset the odom data from the origin: [-0.45, -1.9, 0.0]
        if message.obstacle_status.obstacle_data != 'None':
            return ((message.robot_status.x_coord, message.robot_status.y_coord), message.obstacle_status.obstacle_data)
        else:
            return ((message.robot_status.x_coord, message.robot_status.y_coord), 3)

    def is_start(self, odom_x_coord, odom_y_coord):
        # have to offset the odom data from the origin: [-0.45, -1.9, 0.0]
        x_coord = odom_x_coord + 0.45 
        y_coord = odom_y_coord + 1.9

    def is_goal(self, state, goal):
        return state == goal

    def get_solution(self, world_map, start, goal):
    # world_map = json.load()
    # end = message.goal
    # start = task_handler.get_state_from_message(ssp_state_message)

        rospy.loginfo("Info[task_handler.CASTaskHandler.get_solution]: Instantiating the domain model...")
        # changed DM to accept start and goal of (x, y) instead of text 'S' and 'G'
        DM = DeliveryBotDomain(world_map, start, goal)

        rospy.loginfo("Info[task_handler.CASTaskHandler.get_solution]: Instantiating the autonomy model...")
        AM = AutonomyModel(DM, [0,1,2,3,])

        rospy.loginfo("Info[task_handler.CASTaskHandler.get_solution]: Instantiating the feedback model...")
        HM = FeedbackModel(DM, AM, ['+', '-', '/', None], ['open'])

        rospy.loginfo("Info[task_handler.CASTaskHandler.get_solution]: Instanaitating the CAS model...")
        solution = CAS(DM, AM, HM, persistence = 0.75)
        rospy.loginfo("Info[task_handler.CASTaskHandler.get_solution]: Solving the CAS model...")
        solution.solve()
        state_map = solution.state_map
        policy = solution.pi 
        return policy, state_map

