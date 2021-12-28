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
    # These variables need to be updated throughout the whole script
    global corner_1, corner_2, corner_3, corner_4, turn, has_turned, pose
    
    corners = np.array([[3.5, 7.5], [7.5, 7.5], [7.5, 3.5], [3.5, 3.5]])#set the coordinates that will make the corners of the square
    pose = pose_msg # take the message of the turtle's current pose
    diff = np.zeros(2) #set a 2D matrix with x and y positions [0 0] to store the current location

    if corner_1: 
        diff[1] = corners[0][1] - pose.y # get the current y coordinate and subtract it from the 1st corner's y coordinate
        distance_y = np.sqrt(diff[1]**2)

        if distance_y < threshold: #if the distance_y is less than 0.02
            corner_1 = False #we are no longer at corner 1
            corner_2 = True #we must be at corner 2
            turn = True #turn the turtle

    if corner_2:
        diff[0] = corners[1][0] - pose.x # get the current x coordinate and subtract it from the 2nd corner's x coordinate
        distance_x = np.sqrt(diff[0]**2)

        if distance_x < threshold: #if the distance_y is less than 0.02
            corner_2 = False #we are no longer at corner 2
            corner_3 = True #we must be at corner 3
            turn = True #turn the turtle

    if corner_3:  
        diff[1] = corners[2][1] - pose.y #get the current y coordinate and subtract it from the 3rd corner's y coordinate
        distance_y = np.sqrt(diff[1]**2)

        if distance_y < threshold: #if the distance_y is less than 0.02
            corner_3 = False #we are no longer at corner 3
            corner_4 = True #we must be at corner 4
            turn = True #turn the turtle

    if corner_4:
        diff[0] = corners[3][0] - pose.x #get the current x coordinate and subtract it from the 4th corner's x coordinate
        distance_x = np.sqrt(diff[0]**2)

        if distance_x < threshold: #if the distance_y is less than 0.02
            corner_4 = False #we are no longer at corner 4
            corner_1 = True #we must be at corner 1
            turn = True #turn the turtle

def square_driver():
    global pose, turn
    rp.init_node('turtlesim_driver', anonymous=True) #this tells rospy the name of your node
    topic = rp.Publisher('/turtle1/cmd_vel', Twist, queue_size=1000) #this tells the publisher to keep an eye on the cmd_vel topic
    subscriber = rp.Subscriber('/turtle1/pose', Pose, pose_callback) #this tells rospy the subscribing node in the pose callback
    rate = rp.Rate(1000) #tells the node to run at 1000hz
    direction_x = 1 #set the direction to +1 in the x axis
    angular = 0 #the object shouldn't rotate

    while not rp.is_shutdown(): #while rospy has not been shutdown/terminated
        if corner_1: # turtle came in facing up at 90 degrees and should be facing right at 0 degrees
            if abs(pose.theta - np.pi/2) > threshold_rotation and turn: # if the turtle is facing any other way than 0 degrees 
                direction_x = 0 #don't move
                angular = -1 #keep turning the turtle
            else:
                direction_x = 1 #move the turtle forward
                angular = 0 #don't turn
                turn = False #set turning status to false
        
        if corner_2:# turtle came in facing right at 0 degrees and should be facing right at 0 degrees
            if abs(pose.theta) > threshold_rotation and turn: # if the turtle is facing any other way than 0 degrees 
                direction_x = 0 #don't move
                angular = -1 #keep turning the turtle
            else:
                direction_x = 1 #move the turtle forward
                angular = 0 #don't turn
                turn = False #set turning status to false

        if corner_3: # turtle came in facing down at -90 degrees and should be facing left at 0 degrees
            if abs(pose.theta + np.pi/2) > threshold_rotation and turn: # if the turtle is facing any other way than -180 degrees 
                direction_x = 0
                angular = -1
            else:
                direction_x = 1
                angular = 0
                turn = False

        if corner_4: #turtle should be facing left at 180 degrees but is currently facing 
            if abs(pose.theta - np.pi) > threshold_rotation and turn: # if the turtle is facing any other way than 180 degrees 
                direction_x = 0
                angular = -1
            else:
                direction_x = 1
                angular = 0
                turn = False

        twist = Twist() #reset the turtle
        twist.linear.x = direction_x #make sure the turtle is moving in the defined direction
        twist.angular.z = angular #turn the turtle when set
        topic.publish(twist) #publish the turtle's coordinates
        rate.sleep()

if __name__ == '__main__':
    try:
        square_driver()

    except rp.ROSInterruptException:
        pass