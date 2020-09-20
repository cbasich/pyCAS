#!/usr/bin/env python
import json
import math
import rospy

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from pyCAS.msg import RobotStatus

# Global variables
DISTANCE_THRESHOLD = 0.15
ODOMETRY_MESSAGE = Odometry()


def odometry_message_callback(message):
    global ODOMETRY_MESSAGE
    ODOMETRY_MESSAGE = message


def get_location(topological_map, current_location):
    """
    params:
        Topological map: a dictionary of all of the state's row/col location to x/y coordinates  

    returns:
        Returns the current location (row, col) by comparing the x/y coorindates of odom data and the states within the topological map
        Uses the distance tolerance to allow some room for odom data error 
    """
    location = None
    current_location_pose = current_location.position

    for state in topological_map["states"]:
        pose = topological_map["states"][state]["pose"]
        state_x = pose['x']
        state_y = pose['y']

        horizontal_distance = state_x - current_location_pose.x
        vertical_distance = state_y - current_location_pose.y
        distance = math.sqrt(horizontal_distance ** 2 + vertical_distance ** 2)

        if distance < DISTANCE_THRESHOLD:
            # TODO: Ideally, this parsing logic should be done before we get here
            tmp = state.strip('(').strip(')').split(',')
            location_row = int(tmp[0])
            location_col = int(tmp[1])
            location = (location_row, location_col)
            break
            
    return location


def get_orientation(current_location):
    """
    params:
        Current location according to odom data   

    returns:
        Returns the yaw in radians from quaternion to euler transformation 
    """
    current_location_orientation = current_location.orientation
    quaternion = [current_location_orientation.x, current_location_orientation.y, current_location_orientation.z, current_location_orientation.w]
    (_, _, yaw) = euler_from_quaternion(quaternion)

    return yaw


def main():
    """
    params:
        None

    returns:
        Publishes the robot status message that contains the location (row, col) and orientation (radians)
    """
    rospy.loginfo("Info[robot_status_monitor.main]: Instantiating the robot_status_monitor node...")
    rospy.init_node("robot_status_monitor", anonymous=True)
    rospy.Subscriber('odom', Odometry, odometry_message_callback, queue_size=1)
    publisher = rospy.Publisher("monitor/robot_status", RobotStatus, queue_size=10)

    rate = rospy.Rate(rospy.get_param("/robot_status_monitor/rate"))
    topological_map = json.load(open(rospy.get_param('/CDB_execution_node/topological_map')))

    while not rospy.is_shutdown():
        if ODOMETRY_MESSAGE: 
            current_location = ODOMETRY_MESSAGE.pose.pose
            location_info = get_location(topological_map, current_location)

            # keeps the state constant while the robot is moving from one location to the next 
            if location_info:
                location = location_info
                previous_location = location
            else: 
                location = previous_location
                
            yaw = get_orientation(current_location)

            # populating robot status message 
            message = RobotStatus() 
            message.location_row = location[0]
            message.location_col = location[1]
            message.heading = yaw
            publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
