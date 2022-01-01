#! /usr/bin/env python
import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped

################# IMPORT all the necessary classes and messages (Twist, Transform message, pose)

class Turner:

    def __init__(self):
        
        rospy.init_node('robot_nav')
        self.tf_buffer = tf2_ros.Buffer() #allows the tf2_frames to respond to requests
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster() # We start the broadcaster for the transformations
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer) #Listener - the name says it all
        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10) # #publish robot's velocity
        self.pose_listener = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback) #subscribe to robot's current pose
        self.rate = rospy.Rate(10)

    def publish_goal(self):

        self.goal_map = TransformStamped() #We open an empty transform which we fill with information about the pose

        self.goal_map.header.stamp = rospy.Time.now() #This is a transformation between world and robot
        self.goal_map.header.frame_id = "map" #set the common frame i.e. view 1
        self.goal_map.child_frame_id = "goal" #set the goal frame i.e. view 2
        
  
        self.goal_map.transform.translation.x = self.target[0] #set the target x axis
        self.goal_map.transform.translation.y = self.target[1] #set the target y axis
        self.goal_map.transform.rotation.w = 1 #set the target rotation axis

        self.tf_broadcaster.sendTransform(self.goal_map)  #send this to the transform broadcaster
        print("Goal published")

    def compute_transform(self):

        received = False #the transform hasn't been received yet
        while not received:
            try:
                T = self.tf_buffer.lookup_transform('base_footprint', 'goal',  rospy.Time(0)) # calculate the transform
                received = True

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
        return T

    def turn(self, target):

        self.target = target #set the target variable
        self.publish_goal() #publish the goal
        T_base = self.compute_transform() #compute the angle which the robot has to turn in
        vel = Twist() #set the movement

        while abs(T_base.transform.translation.y)>= 0.05: #if the angle is less than the threshold
            vel.angular.z = 0.5 * T_base.transform.translation.y #set the z angle the robot will move in based on a comfortable speed
            self.vel_publisher.publish(vel) #publish this velocity
            T_base = self.compute_transform() #compute the velocit again
            self.rate.sleep()

    def pose_callback(self, msg):

        self.pose = msg #keep tabs of the robot's movement and angle

if __name__ == '__main__':
    try:
        goal = np.zeros(3)
        goal[0] = float(input("Set your x goal: ")) #input the x axis wanted
        goal[1] = float(input("Set your y goal: ")) #input the y axis wanted
        goal[2] = float(input("Set your orientation goal (in deg): ")) #input the orientation wanted

        Turtle_turner = Turner() #Instantiate the turner 
        Turtle_turner.turn(goal) #pass the goal coordinates and angle

        print("Reached goal!")
    except rospy.ROSInterruptException:
        print("program interrupted before completion")