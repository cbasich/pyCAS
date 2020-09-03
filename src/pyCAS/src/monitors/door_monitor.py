#!/usr/bin/env python
import rospy
from msg import RobotStatus

# TODO: Implement this function. Perceptual logic should go here to determine what type of door
#       is in front of the robot, or else None.
def get_door_type():
    return None


def main():
    rospy.loginfo("Info[door_monitor.main]: Instantiating the door_monitor node...")
    rospy.init_node("door_monitor", anonymous=True)

    publisher = rospy.Publisher("monitor/door_status", RobotStatus)
    rate = rospy.Rate(rospy.get_param("/door_monitor/rate"))

    while not rospy.is_shutdown():
        message = RobotStatus()

        # TODO: Populate the message.

        rospy.loginfo(message)
        publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
