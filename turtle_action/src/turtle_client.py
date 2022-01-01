#! /usr/bin/env python

from __future__ import print_function
import rospy
import actionlib
from geometry_msgs.msg import Pose2D
from turtle_action.msg import TurtleNavigationAction, TurtleNavigationGoal


def draw_square():
    #set the standard pose locations
    poses = [Pose2D(3.5, 7.5, 0), Pose2D(7.5, 7.5, 0), Pose2D(7.5, 3.5, 0), Pose2D(3.5, 3.5, 0)] 

    #subscribe to the move_turtle action client
    client = actionlib.SimpleActionClient('move_turtle', TurtleNavigationAction)
    client.wait_for_server()

    for next_pose in poses:
        #create an empty goal location
        goal = TurtleNavigationGoal()
        goal.target_pose = next_pose # give the target the next_pose location

        client.send_goal(goal)#send this goal to the client

        client.wait_for_result()#wait for the turtle to move to the goal



if __name__ == '__main__':
    try:
        rospy.init_node('square_node')# call the code's node "square_node"
        draw_square()#start the draw_square function
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
