#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np

def navigate(target):

    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
  
    goal.target_pose.pose.orientation.w = compute_quaternion_1D(target[2])[0]
    goal.target_pose.pose.orientation.z = compute_quaternion_1D(target[2])[3]
  
    goal.target_pose.pose.position.x = target[0]
    goal.target_pose.pose.position.y = target[1]

    # Sends the goal to the action server.
    client.send_goal(goal)

    print("Goal passed to server")

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result() 

def compute_quaternion_1D(orientation):

    z = np.deg2rad(orientation)

    cy = np.cos(z*0.5)
    sy = np.sin(z*0.5)
    # ------------ Because we have only 1 axis of rotation, these terms are always 1, but included for completeness -------------------
    cp = np.cos(0)
    cr = np.cos(0)
    # ------------ Because we have only 1 axis of rotation, these terms are always 0, but included for completeness -------------------
    sp = np.sin(0)
    sr = np.sin(0)

    q = np.zeros(4)

    q[0] = cr * cp * cy + sr * sp * sy #W
    q[1] = sr * cp * cy - cr * sp * sy #X, always 0 for us
    q[2] = cr * sp * cy + sr * cp * sy #Y, always 0 for us
    q[3] = cr * cp * sy - sr * sp * cy #Z

    return q

if __name__ == '__main__':
    try:

        rospy.init_node('robot_nav')

        goal = np.zeros(3)

        goal[0] = float(input("Set your x goal: "))
        goal[1] = float(input("Set your y goal: "))
        goal[2] = float(input("Set your orientation goal (in deg): "))

        result = navigate(goal)

        print("Reached goal!")
    except rospy.ROSInterruptException:
        print("program interrupted before completion")