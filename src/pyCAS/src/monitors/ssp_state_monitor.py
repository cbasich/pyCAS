#!/usr/bin/env python3
import rospy
from pyCAS.msg import SSPState, RobotStatus, ObstacleStatus, Interaction

robot_status_message = RobotStatus()
obstacle_status_message = ObstacleStatus()
interaction_status_message = Interaction()

def robot_status_message_callback(message):
    global robot_status_message
    robot_status_message = message

def obstacle_status_message_callback(message):
    global obstacle_status_message
    obstacle_status_message = message

def interaction_status_message_callback(message):
    global interaction_status_message
    interaction_status_message = message

def main():
    rospy.loginfo("Info[ssp_state_monitor.main]: Instantiating the ssp_state_monitor node...")
    rospy.init_node("ssp_state_monitor", anonymous=True)

    rospy.Subscriber("monitor/robot_status", RobotStatus, robot_status_message_callback, queue_size=1)
    rospy.Subscriber("monitor/obstacle_status", ObstacleStatus, obstacle_status_message_callback, queue_size=1)
    rospy.Subscriber("monitor/interaction", Interaction, interaction_status_message_callback, queue_size=1)

    publisher = rospy.Publisher("monitor/ssp_state_monitor", SSPState, queue_size=10)
    rate = rospy.Rate(rospy.get_param("ssp_state_monitor/rate"))

    while not rospy.is_shutdown():

        message = SSPState()
        if robot_status_message and obstacle_status_message:
            # combines all of the messages into one mega SSPState message 
            message.robot_status = robot_status_message
            message.obstacle_status = obstacle_status_message
            message.interaction_status = interaction_status_message
            publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
