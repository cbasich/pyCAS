#!/usr/bin/env python
import rospy, json, math
from pyCAS.msg import RobotStatus
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


odometry_message = None


def odometry_message_callback(message):
    global odometry_message
    odometry_message = message


# TODO: Implement this function
def get_location(topological_map):
    # add one to account for the robot starting at (1, 1) and not (0, 0) on the map
    # need to convert odom data (in meters) to states (in feet)
    # set a tolerance to say when we are in the area of a state  
    distance_tolerance = 0.15
    location = None
   

    for state in topological_map["states"]:
        # this is the odom position for each state
        pose = topological_map["states"][state]["pose"]
        state_x = pose['x']
        state_y = pose['y']
        # tmp = states.strip('(').strip(')').split(',')
        # obstacle_y = (float(tmp[0])-1) *0.3048 # 1.2
        # obstacle_x = (float(tmp[1]) -1 ) * 0.3048 # 0.9

        current_location_pose = odometry_message.pose.pose.position

        horizontal_distance = (state_x - current_location_pose.x) ** 2
        vertical_distance = (state_y - current_location_pose.y) ** 2
        distance = math.sqrt(horizontal_distance + vertical_distance)

        if state == '(3, 1)':
            if distance < 0.5:
                pass
                # print(distance)
        if distance < distance_tolerance:
            tmp = state.strip('(').strip(')').split(',')
            location_row = int(tmp[0])
            location_col = int(tmp[1])
            location = (location_row, location_col)
            
    return location

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
    topological_map = json.load(open(rospy.get_param('/CDB_execution_node/topological_map')))

    # hard coded for debugging
    old_location = (1, 1)
    while not rospy.is_shutdown():
        if odometry_message:  
            location = get_location(topological_map)
            yaw = get_orientation()
            if not location:
                message = RobotStatus()
                message.location_row = old_location[0]
                message.location_col = old_location[1]
                message.heading = yaw
                publisher.publish(message)
            if location:
                message = RobotStatus()
                message.location_row = location[0]
                message.location_col = location[1]
                message.heading = yaw
                publisher.publish(message)
                old_location = location

        rate.sleep()


if __name__ == '__main__':
    main()
