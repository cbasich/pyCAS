#!/usr/bin/env python
import rospy
from pyCAS.msg import RobotStatus
from nav_msgs.msg import Odometry


odometry_message = None


def odometry_message_callback(message):
    global odometry_message
    odometry_message = message


# TODO: Implement this function
def get_location():
    current_x = int(odometry_message.pose.pose.position.x)
    current_y = int(odometry_message.pose.pose.position.y)
    return current_x, current_y


def main():
    rospy.loginfo("Info[robot_status_monitor.main]: Instantiating the robot_status_monitor node...")
    rospy.init_node("robot_status_monitor", anonymous=True)

    rospy.Subscriber('odom', Odometry, odometry_message_callback, queue_size=1)

    publisher = rospy.Publisher("monitor/robot_status", RobotStatus, queue_size=10)
    rate = rospy.Rate(rospy.get_param("/robot_status_monitor/rate"))

    while not rospy.is_shutdown():
        if odometry_message: 
            current_x, current_y = get_location()
            message = RobotStatus()
            message.x_coord = current_x
            message.y_coord = current_y
            # publish the RobotStatus message 
            publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
