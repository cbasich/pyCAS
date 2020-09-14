#!/usr/bin/env python
import rospy
from pyCAS.msg import RobotStatus
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


odometry_message = None


def odometry_message_callback(message):
    global odometry_message
    odometry_message = message


# TODO: Implement this function
def get_location():
    # add one to account for the robot starting at (1, 1) and not (0, 0) on the map
    # need to convert odom data (in meters) to states (in feet)
    current_x = abs(int(round(odometry_message.pose.pose.position.x*3.281))) + 1
    current_y = abs(int(round(odometry_message.pose.pose.position.y*3.281))) + 1
    return current_x, current_y

def get_orientation():
    # the orientation in quaternion
    quat = [odometry_message.pose.pose.orientation.x, odometry_message.pose.pose.orientation.y, odometry_message.pose.pose.orientation.z, odometry_message.pose.pose.orientation.w]
    # the heading is the yaw which is what we want for robot face
    (_, _, yaw) = euler_from_quaternion(quat)
    return yaw



def main():
    rospy.loginfo("Info[robot_status_monitor.main]: Instantiating the robot_status_monitor node...")
    rospy.init_node("robot_status_monitor", anonymous=True)

    rospy.Subscriber('odom', Odometry, odometry_message_callback, queue_size=1)

    publisher = rospy.Publisher("monitor/robot_status", RobotStatus, queue_size=10)
    rate = rospy.Rate(rospy.get_param("/robot_status_monitor/rate"))

    while not rospy.is_shutdown():
        if odometry_message: 
            current_x, current_y = get_location()
            yaw = get_orientation()
            message = RobotStatus()
            message.x_coord = current_x
            message.y_coord = current_y
            message.heading = yaw
            publisher.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()
