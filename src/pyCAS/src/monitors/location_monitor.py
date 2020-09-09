#!/usr/bin/env python3
import json
import math

import rospy
from nav_msgs.msg import Odometry
from pyCAS.msg import Location


DISTANCE_THRESHOLD = 1.0

odometry_message = None


def odometry_message_callback(message):
    global odometry_message
    odometry_message = message


def is_obstacle(world_map):
    nearest_location = None
    nearest_distance = float('inf')

    for location in world_map['locations']:
        location_pose = world_map['locations'][location]['pose']

        current_location_pose = odometry_message.pose.pose.position

        horizontal_distance = (location_pose['x'] - current_location_pose.x) ** 2
        vertical_distance = (location_pose['y'] - current_location_pose.y) ** 2
        distance = math.sqrt(horizontal_distance + vertical_distance)

        if nearest_location is None or distance < nearest_distance:
            nearest_location = location
            nearest_distance = distance

    return nearest_location, nearest_distance


def main():
    rospy.init_node('location_monitor', anonymous=True)
    rospy.loginfo('Info[location_monitor.main]: Instantiating the location_monitor node...')

    rospy.Subscriber('odom', Odometry, odometry_message_callback, queue_size=1)

    publisher = rospy.Publisher('pyCAS/location', Location, queue_size=10)
    rate = rospy.Rate(rospy.get_param('robot_status_monitor/rate'))

    world_map = json.load(open(rospy.get_param('CDB_execution_node/topological_map')))

    current_location = None

    while not rospy.is_shutdown():
        if odometry_message:
            location, distance = is_obstacle(world_map)

            if current_location is None:
                current_location = location

            if distance < DISTANCE_THRESHOLD:
                current_location = location

            message = Location()
            message.location = current_location

            publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
