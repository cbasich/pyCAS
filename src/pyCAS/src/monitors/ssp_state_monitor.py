#!/usr/bin/env python
import rospy
from pyCAS.msg import SSPState, Location, Interaction

location = None
iteraction = None

def location_message_callback(message):
    global location
    location = message


def interaction_message_callback(message):
    global interaction
    interaction = message


def main():
    rospy.loginfo("Info[ssp_state_monitor.main]: Instantiating the ssp_state_monitor node...")
    rospy.init_node("ssp_state_monitor", anonymous=True)

    rospy.Subscriber('py_cas/location', Location, location_message_callback, queue_size=1)
    rospy.Subscriber('py_cas/interaction', Interaction, interaction_message_callback, queue_size=1)

    publisher = rospy.Publisher("py_cas/ssp_state", SSPState, queue_size=10)
    rate = rospy.Rate(rospy.get_param("ssp_state_monitor/rate"))

    while not rospy.is_shutdown():
        if location: 
            message = SSPState()
            #TODO convert the location to x and y coord for SSPState Robot Status
            # TODO: Populate the message.

            publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
