#!/usr/bin/env python
import rospy
from pyCAS.msg import SSPState, RobotStatus, ObstacleStatus, Interaction

def robot_status_message_callback(message):
    global robot_status_message
    robot_status_message = message

def obstacle_status_message_callback(message):
    global obstacle_status_message
    obstacle_status_message = message

def main():
    rospy.loginfo("Info[ssp_state_monitor.main]: Instantiating the ssp_state_monitor node...")
    rospy.init_node("ssp_state_monitor", anonymous=True)

    rospy.Subscriber("monitor/robot_status", RobotStatus, robot_status_message_callback, queue_size=1)
    rospy.Subscriber("monitor/obstacle_status", ObstacleStatus, obstacle_status_message_callback, queue_size=1)

    publisher = rospy.Publisher("monitor/ssp_state_monitor", SSPState, queue_size=10)
    rate = rospy.Rate(rospy.get_param("ssp_state_monitor/rate"))

    while not rospy.is_shutdown():

        message = SSPState()
        message.robot_status = robot_status_message
        message.obstacle_status = obstacle_status_message
        #TODO convert the location to x and y coord for SSPState Robot Status
        # TODO: Populate the message.

        publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
