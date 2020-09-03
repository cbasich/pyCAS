#!/usr/bin/env python
import rospy
from msg import RobotStatus

# TODO: Implement this function
def get_location():
    pass


def main():
    rospy.loginfo("Info[robot_status_monitor.main]: Instantiating the robot_status_monitor node...")
    rospy.init_node("robot_status_monitor", anonymous=True)

    publisher = rospy.Publisher("monitor/robot_status", RobotStatus)
    rate = rospy.Rate(rospy.get_param("/robot_status_monitor/rate"))

    while not rospy.is_shutdown():
        message = RobotStatus()

        # TODO: Populate the message.

        rospy.loginfo(message)
        publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
