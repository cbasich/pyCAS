#!/usr/bin/env python3
import rospy

from pyCAS.msg import Interaction, ObstacleStatus, RobotStatus, SSPState

ROBOT_STATUS_MESSAGE = RobotStatus()
OBSTACLE_STATUS_MESSAGE = ObstacleStatus()
INTERACTION_STATUS_MESSAGE = Interaction()


def robot_status_message_callback(message):
    global ROBOT_STATUS_MESSAGE
    ROBOT_STATUS_MESSAGE = message


def obstacle_status_message_callback(message):
    global OBSTACLE_STATUS_MESSAGE
    OBSTACLE_STATUS_MESSAGE = message


def interaction_status_message_callback(message):
    global INTERACTION_STATUS_MESSAGE
    INTERACTION_STATUS_MESSAGE = message


def main():
    """
    params:
        None

    returns:
        Publishes a combination of the obstacle status data, robot status data, and interaction status data to the SSP State topic 
    """
    rospy.loginfo("Info[ssp_state_monitor.main]: Instantiating the ssp_state_monitor node...")
    rospy.init_node("ssp_state_monitor", anonymous=True)

    rospy.Subscriber("monitor/robot_status", RobotStatus, robot_status_message_callback, queue_size=1)
    rospy.Subscriber("monitor/obstacle_status", ObstacleStatus, obstacle_status_message_callback, queue_size=1)
    rospy.Subscriber("monitor/interaction", Interaction, interaction_status_message_callback, queue_size=1)

    publisher = rospy.Publisher("monitor/ssp_state_monitor", SSPState, queue_size=10)

    rate = rospy.Rate(rospy.get_param("ssp_state_monitor/rate"))

    while not rospy.is_shutdown():
        message = SSPState()
    
        if ROBOT_STATUS_MESSAGE and OBSTACLE_STATUS_MESSAGE and INTERACTION_STATUS_MESSAGE:

            message.robot_status = ROBOT_STATUS_MESSAGE
            message.obstacle_status = OBSTACLE_STATUS_MESSAGE
            message.interaction_status = INTERACTION_STATUS_MESSAGE
            publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
