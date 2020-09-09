#!/usr/bin/env python
import rospy
from pyCAS.msg import SSPState


def main():
    rospy.loginfo("Info[ssp_state_monitor.main]: Instantiating the ssp_state_monitor node...")
    rospy.init_node("ssp_state_monitor", anonymous=True)

    publisher = rospy.Publisher("monitorssp_state_status", SSPState, queue_size=10)
    rate = rospy.Rate(rospy.get_param("/ssp_state_monitor/rate"))

    while not rospy.is_shutdown():
        message = SSPState()

        # TODO: Populate the message.

        rospy.loginfo(message)
        publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
