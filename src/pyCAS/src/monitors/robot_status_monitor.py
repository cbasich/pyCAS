#!/usr/bin/env python
import json
import math
import rospy
import tf 

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from pyCAS.msg import RobotStatus

# Global variables
DISTANCE_THRESHOLD = 0.25
ODOMETRY_MESSAGE = Odometry()


def odometry_message_callback(message):
    global ODOMETRY_MESSAGE
    ODOMETRY_MESSAGE = message


def get_nearest_location(topological_map, current_location, position_transforms):
    """
    params:
        Topological map: a dictionary of all of the state's row/col location to x/y coordinates 
        Current location: the odometry postion from the odom message
        Position transforms: an array of the transform from odom to map reference frame, [x, y, z] 

    returns:
        Returns the current location (row, col) by comparing the distance of the x/y coorindates of states and the waypoints
        Uses the distance tolerance to allow some room for odom data error 
    """
    location = None
    current_location_pose = current_location.position

    for state in topological_map["states"]:
        # get the waypoint position for each state - in map reference frame
        pose = topological_map["states"][state]["position"]
        map_state_x = pose['x']
        map_state_y = pose['y']

        # transform from odom to map data using the tf listener 
        map_current_location_x = current_location_pose.x + position_transforms[0]
        map_current_location_y = current_location_pose.y + position_transforms[1]

        # calculate the distance 
        horizontal_distance = map_state_x - map_current_location_x
        vertical_distance = map_state_y - map_current_location_y
        distance = math.sqrt(horizontal_distance ** 2 + vertical_distance ** 2)

        # compare the distance to the distance threshold
        if distance < DISTANCE_THRESHOLD:
            # parse the state from the topological map - '(row, col)' to (int(row), int(col)
            tmp = state.strip('(').strip(')').split(',')
            location_row = int(tmp[0])
            location_col = int(tmp[1])
            location = (location_row, location_col)
            break
            
    return location


def get_orientation(current_location, quaternion_transforms):
    """
    params:
        Current location according to odom data   
        Quaternion transforms is the transform from the odom to the map

    returns:
        Returns the yaw in radians from quaternion to euler transformation 
    """

    # gets
    current_location_orientation = current_location.orientation
    # transform from odom to map 
    map_orientation_x = current_location_orientation.x + quaternion_transforms[0]
    map_orientation_y = current_location_orientation.y + quaternion_transforms[1]
    map_orientation_z = current_location_orientation.z + quaternion_transforms[2]
    map_orientation_w = current_location_orientation.w + quaternion_transforms[3]

    # transform from quaternion to euler 
    quaternion = [map_orientation_x, map_orientation_y, map_orientation_z, map_orientation_w]
    (_, _, yaw) = euler_from_quaternion(quaternion)

    return yaw


def main():
    rospy.loginfo("Info[robot_status_monitor.main]: Instantiating the robot_status_monitor node...")
    rospy.init_node("robot_status_monitor", anonymous=True)

    # subscribe to the odom data 
    rospy.Subscriber('odom', Odometry, odometry_message_callback, queue_size=1)
    # publish the robot status 
    publisher = rospy.Publisher("monitor/robot_status", RobotStatus, queue_size=10)

    rate = rospy.Rate(rospy.get_param("/robot_status_monitor/rate"))
    topological_map = json.load(open(rospy.get_param('/CDB_execution_node/topological_map')))
    listener = tf.TransformListener()

    while not rospy.is_shutdown():

        # listening into tf transforms to find the transform between map and odom
        try:
            (position_transforms, quaternion_transforms) = listener.lookupTransform('/map', '/odom', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        if ODOMETRY_MESSAGE: 
            # get the current location with reference to the odom data 
            current_location = ODOMETRY_MESSAGE.pose.pose
            # get the nearest location bsaed off of waypoints and odom/map data
            location_info = get_nearest_location(topological_map, current_location, position_transforms)

            # keeps the state constant while the robot is moving from one location to the next 
            if location_info:
                location = location_info
                previous_location = location
            else: 
                location = previous_location
            
            # get the orientation of the robot
            yaw = get_orientation(current_location, quaternion_transforms)

            # populating robot status message 
            message = RobotStatus() 
            message.location_row = location[0]
            message.location_col = location[1]
            message.heading = yaw
            publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
