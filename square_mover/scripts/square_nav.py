#!/usr/bin/env python

import rospy as rp
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
# Import all necessary messages such as Twist and Pose from ROS and turtlesim messages

corner_1 = False
corner_2 = True
corner_3 = False
corner_4 = False
turn = False
threshold = 0.02
threshold_rotation = 0.01
pose = Pose()

def pose_callback(pose_msg):
    global corner_1, corner_2, corner_3, corner_4, turn, has_turned, pose
    # These variables need to be updated throughout the whole script

    corners = np.array([[3.5, 7.5], [7.5, 7.5], [7.5, 3.5], [3.5, 3.5]])
    pose = pose_msg
    diff = np.zeros(2)

    if corner_1: 
        diff[1] = corners[0][1] - pose.y
        distance_y = np.sqrt(diff[1]**2)

        if distance_y < threshold:
            corner_1 = False
            corner_2 = True
            turn = True

    if corner_2:
        diff[0] = corners[1][0] - pose.x   
        distance_x = np.sqrt(diff[0]**2)

        if distance_x < threshold:
            corner_2 = False
            corner_3 = True
            turn = True

    if corner_3:  
        diff[1] = corners[2][1] - pose.y
        distance_y = np.sqrt(diff[1]**2)

        if distance_y < threshold:
            corner_3 = False
            corner_4 = True
            turn = True

    if corner_4:
        diff[0] = corners[3][0] - pose.x   
        distance_x = np.sqrt(diff[0]**2)

        if distance_x < threshold:
            corner_4 = False
            corner_1 = True
            turn = True

def square_driver():
    global pose, turn
    rp.init_node('turtle_driver', anonymous=True)
    topic = rp.Publisher('/turtle1/cmd_vel', Twist, queue_size=1000)
    subscriber = rp.Subscriber('/turtle1/pose', Pose, pose_callback)
    rate = rp.Rate(1000)
    direction_x = 1
    angular = 0

    while not rp.is_shutdown():
        if corner_1:
            if abs(pose.theta - np.pi/2) > threshold_rotation and turn:
                direction_x = 0
                angular = -1
            else:
                direction_x = 1
                angular = 0
                turn = False
        
        if corner_2:
            if abs(pose.theta) > threshold_rotation and turn:
                direction_x = 0
                angular = -1
            else:
                direction_x = 1
                angular = 0
                turn = False

        if corner_3:
            if abs(pose.theta + np.pi/2) > threshold_rotation and turn:
                direction_x = 0
                angular = -1
            else:
                direction_x = 1
                angular = 0
                turn = False

        if corner_4:
            if abs(pose.theta - np.pi) > threshold_rotation and turn:
                direction_x = 0
                angular = -1
            else:
                direction_x = 1
                angular = 0
                turn = False

        twist = Twist()
        twist.linear.x = direction_x
        twist.angular.z = angular
        topic.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        square_driver()

    except rp.ROSInterruptException:
        pass
