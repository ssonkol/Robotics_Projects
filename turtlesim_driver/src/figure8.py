#!/usr/bin/env python

import rospy as rp
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
# Import all necessary messages such as Twist and Pose from ROS and turtlesim messages

change = False
just_changed = True
has_initial_pose = False
threshold = 0.02
initial_pose = Pose()

def pose_callback(pose_msg):
    global change, just_changed, threshold, has_initial_pose, initial_pose
    # These variables need to be updated throughout the whole script

    if has_initial_pose is False:
        initial_pose = pose_msg
        has_initial_pose = True
    # We want to capture the initial pose (crossing point) to which we will calculate our distance. We only need to do this once.

    pose = pose_msg
    diff = np.zeros(2)

    diff[0] = initial_pose.x - pose.x
    diff[1] = initial_pose.y - pose.y
    distance = np.sqrt(diff[0]**2 + diff[1]**2)

    if just_changed is False and distance < threshold:
        change = True
        just_changed = True

    elif just_changed is True and distance > threshold:
        just_changed = False

def figure8_driver():
    global change, just_changed

    rp.init_node('turtlesim_driver', anonymous=True)
    topic = rp.Publisher('/turtle1/cmd_vel', Twist, queue_size=1000)
    subscriber = rp.Subscriber('/turtle1/pose', Pose, pose_callback)
    #In addition to the publisher for the simple driver, we also need a subscriber to obtain the pose from turtlesim
    rate = rp.Rate(10)

    direction = -1

    while not rp.is_shutdown():
        if change is True:
            direction = -1 * direction
            change = False

        twist = Twist()
        twist.linear.x = 1
        twist.angular.z = direction
        topic.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        figure8_driver()

    except rp.ROSInterruptException:
        pass