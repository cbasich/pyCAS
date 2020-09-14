#!/usr/bin/env python3
import json
import os
import rospy
import roslib
import actionlib
from geometry_msgs.msg import Point, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from pyCAS.msg import RobotStatus, ObstacleStatus, SSPState, TaskRequest, Interaction
from task_handler import CASTaskHandler

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
    # this has the waypoints that have obstacles  
    world_map = json.load(open(obstacle_map))
    # this handles the CAS model 
    task_handler = CASTaskHandler()
    
    # start position from odom data - wait for the init delay
    while not ssp_state_message:
        rospy.loginfo('Info[CDB_execution_node.execute]: waiting to get start position...')
        rospy.sleep(wait_duration)
   
    start_x = ssp_state_message.robot_status.x_coord
    start_y = ssp_state_message.robot_status.y_coord

    start = (start_x, start_y)
    goal = (message.goal_x, message.goal_y)

    model = task_handler.get_problem(grid_map, start, goal)
    policy, state_map = task_handler.get_solution(model)
    rospy.loginfo("Info[CDB_execution_node.instantiate]: Retrieved solution...")
    
    current_state = None
    
    while not task_handler.is_goal(current_state, goal):
        # state is in format ((x, y, theta), obstacle_status)
        new_state = task_handler.get_state(ssp_state_message)
        print("This is new state : ")
        print(new_state)
        print("This is current state : ")
        print(current_state)
        if new_state != current_state:
            current_state = new_state
            state_index = state_map[current_state]
            current_action = policy[state_index]
            
            # for debug
            # print(current_state)
            # print(current_action)
            
            if current_action[1] != 3: 
                # TODO human interaction handling 
                # LoA 0: Human does the action
                if current_action[1] == 0:
                    rospy.loginfo("Level 0 Autonomy: Will wait for human interference to remove obstacle... ")
                    response = input("Have you removed the obstacle? [y/n]: ")
                    if response[0] == 'y' or response[0] == 'Y':
                        interaction = Interaction()
                        interaction.status = 'open'
                        INTERACTION_PUBLISHER.publish(interaction)
                    else:
                        rospy.loginfo("Error: no human interaction to move the obstacle... please try again later")
                # LoA 1: Robot requests permission
                elif current_action[1] == 1:
                    rospy.loginfo("Level 1 Autonomy: Ask permission to remove obstacle... ")
                    response = input("May I open the door? [y/n]: ")
                    if response[0] == 'y' or response[0] == 'Y':
                        input('Press any key to confirm that the door has been opened: ')
                        interaction = Interaction()
                        interaction.status = 'open'
                        INTERACTION_PUBLISHER.publish(interaction)
                    # request permission
                    elif response[0] == 'n' or response[0] == 'N':
                        rospy.loginfo('Forcing robot to turn around... ')
                    else:
                        rospy.loginfo("Invalid input")
                # LoA 2: Robot acts under supervision
                elif current_action[1] == 2:
                    response = input('Are you supervising my actions?')
                    rospy.loginfo('Level 2 Autonomy: Not sure what to do ?')
                    if response[0] == 'y' or response[0] == 'Y':
                        interaction = Interaction()
                        interaction.status = 'open'
                        INTERACTION_PUBLISHER.publish(interaction)   
            else: 
                # action is in format of row, col NOT x, y 
                # adding action to current state = x + action[1], y + action[0]
                # convert foot to meters * 0.3048
                target_state = (((current_state[0][1]-1)*0.3048 + current_action[0][1]*0.3048), -1*((current_state[0][0]-1)*0.3048 + current_action[0][0]*0.3048))
                x = target_state[0]
                y = target_state[1]
                # for debugging 
                # x = 0.3
                # y = -1.2
                print(target_state)
                next_location = MoveBaseGoal()
                next_location.target_pose.header.frame_id = 'map'
                next_location.target_pose.header.stamp = rospy.Time.now()
                next_location.target_pose.pose = Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1))
                print(next_location)
                print('----------------------------------------------')
                NAVIGATION_SERVICE.send_goal(next_location)

                status = NAVIGATION_SERVICE.wait_for_result()
                if not status:
                    raise RuntimeError('Failed to reach the action server')
            
        rospy.sleep(wait_duration)
    rospy.loginfo('Info[CDB_execution_node.execute]: Completed the task')


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