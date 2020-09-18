#!/usr/bin/env python3
import json
import math

import rospy
from nav_msgs.msg import Odometry

from pyCAS.msg import Interaction, ObstacleStatus, RobotStatus

# A distance threshold in meters that indicates when the robot is near an obstacle
DISTANCE_THRESHOLD = 0.15

odometry_message = Odometry()
interaction_message = Interaction()


def odometry_message_callback(message):
    global odometry_message
    odometry_message = message


def interaction_message_callback(message):
    global interaction_message
    interaction_message = message


# TODO: Implement this function
def get_door_type():
    return None


def get_nearest_obstacle(obstacle_map, topological_map):
    nearest_obstacle_location = None
    nearest_obstacle_data = "None"

    for obstacle_location in obstacle_map.keys():
        obstacle_pose = topological_map["states"][obstacle_location]["pose"]
        obstacle_x = obstacle_pose["x"]
        obstacle_y = obstacle_pose["y"]
        
        current_location_pose = odometry_message.pose.pose.position

        horizontal_distance = obstacle_x - current_location_pose.x
        vertical_distance = obstacle_y - current_location_pose.y
        distance = math.sqrt(horizontal_distance ** 2 + vertical_distance ** 2)

        if distance < DISTANCE_THRESHOLD:
            nearest_obstacle_location = obstacle_location
            nearest_obstacle_data = str(obstacle_map[nearest_obstacle_location]["doortype"])
            break
            
    return nearest_obstacle_location, nearest_obstacle_data


def main():
    rospy.loginfo("Info[obstacle_monitor.main]: Instantiating the obstacle_monitor node...")
    rospy.init_node("obstacle_monitor", anonymous=True)
    
    rospy.Subscriber('odom', Odometry, odometry_message_callback, queue_size=1)
    rospy.Subscriber("monitor/interaction", Interaction, interaction_message_callback, queue_size=1)

    publisher = rospy.Publisher("monitor/obstacle_status", ObstacleStatus, queue_size=10)

    rate = rospy.Rate(rospy.get_param("/obstacle_monitor/rate"))
    obstacle_map = json.load(open(rospy.get_param('/CDB_execution_node/obstacle_map')))
    topological_map = json.load(open(rospy.get_param('/CDB_execution_node/topological_map')))

    while not rospy.is_shutdown():
        if odometry_message:
            _, nearest_obstacle_data = get_nearest_obstacle(obstacle_map, topological_map)

            # TODO: This logic looks like it can be simplified
            message = ObstacleStatus()
            message.obstacle_data = nearest_obstacle_data
            if nearest_obstacle_data != "None":
                if interaction_message:
                    if interaction_message.status == 'open':
                        door_status = 'open'
                else:
                    door_status = 'closed'
                message.door_status = door_status

            publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
