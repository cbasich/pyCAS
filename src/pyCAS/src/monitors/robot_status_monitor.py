#!/usr/bin/env python
import json
import math

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from pyCAS.msg import RobotStatus

# A distance threshold in meters that indicates when the robot is near a state
DISTANCE_THRESHOLD = 0.15

odometry_message = Odometry()


def odometry_message_callback(message):
    global odometry_message
    odometry_message = message


def get_location(topological_map, previous_location):
    location = previous_location
   
    for state in topological_map["states"]:
        pose = topological_map["states"][state]["pose"]
        state_x = pose['x']
        state_y = pose['y']

        current_location_pose = odometry_message.pose.pose.position

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


def get_orientation():
    quaternion = [odometry_message.pose.pose.orientation.x, odometry_message.pose.pose.orientation.y, odometry_message.pose.pose.orientation.z, odometry_message.pose.pose.orientation.w]
    (_, _, yaw) = euler_from_quaternion(quaternion)
    return yaw


def main():
    rospy.loginfo("Info[robot_status_monitor.main]: Instantiating the robot_status_monitor node...")
    rospy.init_node("robot_status_monitor", anonymous=True)

    rospy.Subscriber('odom', Odometry, odometry_message_callback, queue_size=1)

    publisher = rospy.Publisher("monitor/robot_status", RobotStatus, queue_size=10)

    rate = rospy.Rate(rospy.get_param("/robot_status_monitor/rate"))
    topological_map = json.load(open(rospy.get_param('/CDB_execution_node/topological_map')))

    # TODO: Set this to the start state of the robot
    previous_location = (1, 1)

    while not rospy.is_shutdown():
        if odometry_message:  
            location = get_location(topological_map, previous_location)
            yaw = get_orientation()

            # TODO: Confirm that this is correct since the robot changed its path
            message = RobotStatus() 
            message.location_row = location[0]
            message.location_col = location[1]
            message.heading = yaw
            publisher.publish(message)

            previous_location = location

        rate.sleep()


if __name__ == '__main__':
    main()
