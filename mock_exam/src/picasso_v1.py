#!/usr/bin/env python
# coding=utf-8
import random
import rospy
import sys
from nav_msgs.msg import Path #note : add path_msg to cmakeList.txt and .xml
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class PicassoNavigation :
    def __init__(self):
        self.machine_pose = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_cb)#subscribe to amcl_pose topic
        self.machine_path = rospy.Publisher('/nav_msg/Path', Path, queue_size=1) #publish to the topic /nav_msg/Path 
        self.path_history = Path()#create a Path object to store path history and pass to self.machine_path when we publish in amcl_cb
        self.path_history.header.frame_id = "map" #define our frame_id as it is needed in our Path publisher to tell /nav_msg/Path which frame/map to draw on
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction) #set our actionlib client
        self.move_base_client.wait_for_server()#don't do anything till our server is created

    #generate random positions
        #store these in a list
    #Make robot go to positions
        #Store amcl data
    #do we need to send this info to another topic - how does rviz read the acml history

    def picasso_drawer(self):
        #self.amcl_pose_data = msg.data
        while True:
            #generate -> move ->generate agin...
            coordinate = [(random.randint(0,13)), (random.randint(0,10)), 1] #randomly set our pose coordinates in the form [x-axis, y-axis, orientation]
            goal = MoveBaseGoal() #instanciate our MoveBaseGoal class to send the goal
            goal.target_pose.header.frame_id = 'map' #does the same thing as self.patth_history - we could possibly get rid of this
            goal.target_pose.pose.position.x = float(coordinate[0]) #set our x coordinate
            goal.target_pose.pose.position.y = float(coordinate[1]) #set our y coordinate
            goal.target_pose.pose.orientation.w = float(coordinate[2])  #set our orientation
            self.move_base_client.send_goal(goal)#send the goal to our MoveBaseAction client
            self.move_base_client.wait_for_result()#don't do anything till the machine has arrived at the goal


    def amcl_cb(self,msg):
        pose = PoseStamped() #initialises a pose object (essentially the format a pose is needed to be in to be passed on to path)
        pose.header = msg.header #set the pose header as the same message header
        pose.pose = msg.pose.pose #get the message pose data and add it to the Pose object's own pose section
        self.path_history.poses.append(pose)  #Append the pose object to our Path history
        self.machine_path.publish(self.path_history)#publish the updated version of path history

    def main_loop(self):#have the program loop through this
        rate = rospy.Rate(10)#set a rate of 10hz
        while not rospy.is_shutdown(): #whilst the terminal hasn't shutdown the process
            picasso_drawer()#call the picasso drawer
            rate.sleep()#sleep

if __name__ == "__main__":
    rospy.init_node('picasso_nav', sys.argv) #initialise the node
    navigation_system = PicassoNavigation() #instantiate the class
    navigation_system.picasso_drawer() #start the drawer
    rospy.spin()
