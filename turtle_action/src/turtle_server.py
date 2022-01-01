#! /usr/bin/env python

import math
import angles
import numpy as np
import actionlib
import rospy
from turtle_action.msg import TurtleNavigationAction, TurtleNavigationFeedback
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleServer:

    def __init__(self):
        self._as = actionlib.SimpleActionServer('move_turtle', TurtleNavigationAction, execute_cb=self.go_to_goal,
                                                auto_start=False) #subscribe to the action server move_turtle
        self._as.start()
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) #set the turtle's velocity to a publisher
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose) #set the turtle's location/pose to the subscriber
        self.pose = Pose() #start the turtle off with a simple pose

    def go_to_goal(self, goal):

        d = np.zeros(2) #set the distance with [0 0]
        p = np.zeros(2) #set the pose with [0 0]
        g = np.zeros(2) #set the goal with [0 0]

        p[0] = self.pose.x #Assign the current x pose with the x coordinate
        p[1] = self.pose.y #Assign the current y pose with the y coordinate

        g[0] = goal.target_pose.x #Assign the target x pose with the target x coordinate
        g[1] = goal.target_pose.y #Assign the target y pose with the target y coordinate

        d = g - p # get the distance matrix

        distance = np.linalg.norm(d) #get the frobinus norm of the distance
        rate = rospy.Rate(10) #set the rate to 10hz

        threshold = 0.05

        #whilst the turtle is active and the set distance is more than the threshold (i.e it has not reached its target)
        while not rospy.is_shutdown() and self._as.is_active() and distance > threshold: 
            angle = math.atan2(d[1], d[0]) #calculate the current angle of the distance matrix
            angle_difference = angles.shortest_angular_distance(self.pose.theta, angle) #work out the difference between the current angle and distance angle

            twist = Twist()#set the current movement

            twist.angular.z = 2 * angle_difference #turn as fast as the angle difference
            if -0.0174533 < angle_difference < 0.0174533:
                twist.linear.x = distance

            self.velocity_publisher.publish(twist) #send this back to the velocity publisher

            feedback = TurtleNavigationFeedback()#get the feedback topic
            feedback.distance = distance #give the distance variable
            self._as.publish_feedback(feedback)# publish this feedback

            rate.sleep()

            p[0] = self.pose.x #get the new x pose
            p[1] = self.pose.y #get the new y pose

            g[0] = goal.target_pose.x #get the new target x pose
            g[1] = goal.target_pose.y #get the new target y pose

            d = g - p # set the new distance between the new current pose and new target pose

            distance = np.linalg.norm(d) #normalise this distance

        if distance <= threshold: #if the object is at its intended goal
            self._as.set_succeeded()  #object has  reached its goal
        else:
            self._as.set_preempted()#the object was interrupted when it was on its way to the goal (i.e. if it was blocked)

    def update_pose(self, data):
        self.pose = data #keep an tabs on the current pose of the object


if __name__ == '__main__':
    rospy.init_node('turtle_navigation') #call the script's node "turtle_navigation"
    server = TurtleServer() #add the turtle server
    rospy.spin()
