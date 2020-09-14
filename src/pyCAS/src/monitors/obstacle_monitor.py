#!/usr/bin/env python3
import rospy
import math
import json
from pyCAS.msg import RobotStatus, ObstacleStatus, Interaction
from nav_msgs.msg import Odometry


odometry_message = None
interaction_status_message = None

def odometry_message_callback(message):
    global odometry_message
    odometry_message = message

def interaction_status_message_callback(message):
    global interaction_status_message
    interaction_status_message = message

# TODO: Implement this function. Perceptual logic should go here to determine what type of door
#       is in front of the robot, or else None.
def get_door_type():
    return None

def get_obstacle_status(obstacle_map):
    # set a tolerance to say when we are in the area of an obstacle 
    # NEED TO CONVERT EVERYTHING INTO METERS
    distance_tolerance = 0.15
    obstacle_location = None
    obstacle_data = 'None'

    for obstacle in obstacle_map.keys():
        tmp = obstacle.strip('(').strip(')').split(',')
        obstacle_y = (float(tmp[0])-1) *0.3048 # 1.2
        obstacle_x = (float(tmp[1]) -1 ) * 0.3048 # 0.9

        current_location_pose = odometry_message.pose.pose.position

        horizontal_distance = (obstacle_x - abs(current_location_pose.x)) ** 2
        vertical_distance = (obstacle_y - abs(current_location_pose.y)) ** 2
        distance = math.sqrt(horizontal_distance + vertical_distance)

        if distance < distance_tolerance:
            obstacle_location = obstacle
            obstacle_data = str(obstacle_map[obstacle])
            
    return obstacle_location, obstacle_data


def main():
    rospy.loginfo("Info[obstacle_monitor.main]: Instantiating the obstacle_monitor node...")
    rospy.init_node("obstacle_monitor", anonymous=True)
    
    rospy.Subscriber('odom', Odometry, odometry_message_callback, queue_size=1)

    rospy.Subscriber("monitor/interaction", Interaction, interaction_status_message_callback, queue_size=1)

    publisher = rospy.Publisher("monitor/obstacle_status", ObstacleStatus, queue_size=10)
    rate = rospy.Rate(rospy.get_param("/obstacle_monitor/rate"))
    obstacle_map = json.load(open(rospy.get_param('/CDB_execution_node/obstacle_map')))

    while not rospy.is_shutdown():
        if odometry_message:
            _, obstacle_data = get_obstacle_status(obstacle_map)

            message = ObstacleStatus()
            message.obstacle_data = obstacle_data
            if obstacle_data != 'None':
                cntr = 0
                if interaction_status_message:
                    if interaction_status_message.status == 'open':
                        door_status = 'open'
                else:
                    door_status = 'closed'
                message.door_status = door_status

            publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
