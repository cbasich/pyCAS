#!/usr/bin/env python3
import json
import math
import rospy

from nav_msgs.msg import Odometry
from pyCAS.msg import Interaction, ObstacleStatus, RobotStatus
from IPython import embed

# GLobal variables
DISTANCE_THRESHOLD = 0.15
ODOMETRY_MESSAGE = Odometry()
INTERACTION_MESSAGE = Interaction()


def odometry_message_callback(message):
    global ODOMETRY_MESSAGE
    ODOMETRY_MESSAGE = message


def interaction_message_callback(message):
    global INTERACTION_MESSAGE
    INTERACTION_MESSAGE = message


def get_nearest_obstacle(obstacle_map, topological_map):
    """
    params:
        Obstacle map: a dictionary mapping the row/col location of each obstacle to the relavant obstacle data (e.g. door type)
        Topological map: a dictionary of all of the state's row/col location to x/y coordinates  

    returns:
        Obstacle door type: returns either 'push' or 'pull' if there is an obstacle nearby 
    """
    obstacle_door_type = "None"

    for obstacle_location in obstacle_map.keys():
        # find the obstacle data from loaded obstacle json file
        obstacle_pose = topological_map["states"][obstacle_location]["pose"]
        obstacle_x = obstacle_pose["x"]
        obstacle_y = obstacle_pose["y"]
        
        current_location_pose = ODOMETRY_MESSAGE.pose.pose.position

        # calculate the distance from obstacle
        horizontal_distance = obstacle_x - current_location_pose.x
        vertical_distance = obstacle_y - current_location_pose.y
        distance = math.sqrt(horizontal_distance ** 2 + vertical_distance ** 2)

        # check to see if the distance is within the current threshold 
        if distance < DISTANCE_THRESHOLD:
            obstacle_door_type = obstacle_map[obstacle_location]["doortype"]
            break
  
    return obstacle_door_type


def main():
    """
    params:
        None

    returns:
        Publishes an obstacle status message that contains door type and door status
        Door status is updated by the interaction topic 
        Door type is from the location of the current position (from odom) compared to the obstacle dictionary
    """
    rospy.loginfo("Info[obstacle_monitor.main]: Instantiating the obstacle_monitor node...")
    rospy.init_node("obstacle_monitor", anonymous=True)
    
    rospy.Subscriber('odom', Odometry, odometry_message_callback, queue_size=1)
    rospy.Subscriber("monitor/interaction", Interaction, interaction_message_callback, queue_size=1)

    publisher = rospy.Publisher("monitor/obstacle_status", ObstacleStatus, queue_size=10)

    rate = rospy.Rate(rospy.get_param("/obstacle_monitor/rate"))
    obstacle_map = json.load(open(rospy.get_param('/CDB_execution_node/obstacle_map')))
    topological_map = json.load(open(rospy.get_param('/CDB_execution_node/topological_map')))

    while not rospy.is_shutdown():
        if ODOMETRY_MESSAGE:
            # determine the nearest obstacle and gets relevant data 
            obstacle_door_type = get_nearest_obstacle(obstacle_map, topological_map)
            message = ObstacleStatus()
            message.obstacle_data = obstacle_door_type

            if obstacle_door_type != "None":
                door_status = "None"

                if INTERACTION_MESSAGE and INTERACTION_MESSAGE.status == 'open':
                    door_status = 'open'

                else:
                    door_status = 'closed'
                message.door_status = door_status

            publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
