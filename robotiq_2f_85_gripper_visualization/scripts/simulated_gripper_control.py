#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from robotiq_2f_85_gripper_visualization.srv import Control2F85

MIN_ROTATION = 0.0
MAX_ROTATION = 0.8
MIN_MM = 0
MAX_MM = 100

LEFT_CONTROLLER_NAME = "/robotiq_2f_85/left_controller/command"
RIGHT_CONTROLLER_NAME = "/robotiq_2f_85/right_controller/command"

publisher_left = False
publisher_right = False

def get_rotation(distance):
    """ Calculate the rotation for a distance """
    return ((MAX_MM - float(distance) - MIN_MM) / MAX_MM) * (abs(MIN_ROTATION) + MAX_ROTATION) + MIN_ROTATION

def control_gripper(request):
    """ Called when the service is called """
    global publisher_left, publisher_right
    distance = request.distance

    if distance < MIN_MM or distance > MAX_MM:
        rospy.logwarn("%f is an invalid value. Should be between %f and %f", distance, MIN_MM, MAX_MM)
        return -1
    
    rotate_to = get_rotation(distance)

    msg_left = Float64MultiArray(data=[rotate_to, rotate_to, -rotate_to])
    msg_right = Float64MultiArray(data=[rotate_to, rotate_to, -rotate_to])

    if publisher_right and publisher_left:
        publisher_right.publish(msg_right)
        publisher_left.publish(msg_left)

        rospy.loginfo("Rotating to %f radians", rotate_to)
        return rotate_to
    else:
        rospy.logerror("Publishers have failed!")
        return -1

def init_server():
    global publisher_right, publisher_left
    rospy.init_node("control_robotiq_2f_85_server")
    service = rospy.Service('control_robotiq', Control2F85, control_gripper)

    publisher_left = rospy.Publisher(LEFT_CONTROLLER_NAME, Float64MultiArray, queue_size=3)
    publisher_right = rospy.Publisher(RIGHT_CONTROLLER_NAME, Float64MultiArray, queue_size=3)

    rospy.loginfo("Ready to control the Robotiq 2F 85 gripper")
    rospy.spin()


if __name__ == "__main__":
    init_server()