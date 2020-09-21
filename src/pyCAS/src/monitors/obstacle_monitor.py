#!/usr/bin/env python
import json
import math
import rospy
import tf

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


def get_nearest_obstacle(obstacle_map, topological_map, position_transforms):
    """
    params:
        Obstacle map: a dictionary mapping the row/col location of each obstacle to the relavant obstacle data (e.g. door type)
        Topological map: a dictionary of all of the state's row/col location to x/y coordinates  
        Position transforms: an array of the transform from odom to map reference frame, [x, y, z] 

    returns:
        Obstacle door type: returns either 'push' or 'pull' if there is an obstacle nearby 
    """

    obstacle_door_type = "None"

    for obstacle_location in obstacle_map.keys():
        # find the obstacle data from obstacle state dictionary
        obstacle_pose = topological_map["states"][obstacle_location]["position"]
        # position is in map reference fram
        obstacle_x = obstacle_pose["x"]
        obstacle_y = obstacle_pose["y"]
        
        # get the current odometry data 
        current_location_pose = ODOMETRY_MESSAGE.pose.pose.position

        # transform between the odom data to the map refernce by using tf listener 
        map_current_location_x = current_location_pose.x + position_transforms[0]
        map_current_location_y = current_location_pose.y + position_transforms[1]

        # calculate the distance from obstacle
        horizontal_distance = obstacle_x - map_current_location_x
        vertical_distance = obstacle_y - map_current_location_y
        distance = math.sqrt(horizontal_distance ** 2 + vertical_distance ** 2)

        # check to see if the distance is within the current threshold 
        if distance < DISTANCE_THRESHOLD:
            obstacle_door_type = obstacle_map[obstacle_location]["doortype"]
            break
  
    return obstacle_door_type


def main():
    rospy.loginfo("Info[obstacle_monitor.main]: Instantiating the obstacle_monitor node...")
    rospy.init_node("obstacle_monitor", anonymous=True)
    
    # subscribe to odom data topic
    rospy.Subscriber('odom', Odometry, odometry_message_callback, queue_size=1)
    # subscribed to the human interaction topic - checks to see if door is opened
    rospy.Subscriber("monitor/interaction", Interaction, interaction_message_callback, queue_size=1)
    # publishes the door status and door type 
    publisher = rospy.Publisher("monitor/obstacle_status", ObstacleStatus, queue_size=10)

    rate = rospy.Rate(rospy.get_param("/obstacle_monitor/rate"))
    obstacle_map = json.load(open(rospy.get_param('/CDB_execution_node/obstacle_map')))
    topological_map = json.load(open(rospy.get_param('/CDB_execution_node/topological_map')))

    listener = tf.TransformListener()

    while not rospy.is_shutdown():

        # listening into tf transforms to find the transform between map and odom
        try:
            (position_transforms, quaternion_transforms) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
           
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        if ODOMETRY_MESSAGE:
            # determine the nearest obstacle based on waypoints and odom/map data
            obstacle_door_type = get_nearest_obstacle(obstacle_map, topological_map, position_transforms)
            
            # populates the obstacle status message with door type
            message = ObstacleStatus()
            message.obstacle_data = obstacle_door_type

            # populates the obstacle status message with door status 
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
