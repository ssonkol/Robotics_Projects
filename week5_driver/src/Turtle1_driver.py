#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
import tf2_ros
import tf_conversions

################# IMPORT all the necessary classes and messages (Twist, Transform message, pose)

def pose_callback(msg):
    global pose  # Pose needs to be updated throughout the whole script
    pose = msg #set the current pose to the callbacks recieved pose
    T_1 = TransformStamped() # We open an empty transform which we fill with information about the pose
    tf_broadcast = tf2_ros.TransformBroadcaster() # We start the broadcaster for the transformations

    T_1.header.stamp = rospy.Time.now() # This is a transformation between world and turtle 1
    T_1.header.frame_id = "world" #set the common frame i.e. view 1
    T_1.child_frame_id = "turtle1" #set the turtle's fram i.e. view 3 - Turtle 2's perspective
    T_1.transform.translation.x = pose.x #set the current x axis
    T_1.transform.translation.y = pose.y #set the current y axis
    
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, pose.theta)# We use the conversion from Euler to quaternion
    T_1.transform.rotation.x = q[0] #set the x axis
    T_1.transform.rotation.y = q[1] #set the y axis
    T_1.transform.rotation.z = q[2] #set the z axis
    T_1.transform.rotation.w = q[3] #set the w axis - turn angle

    tf_broadcast.sendTransform(T_1) #send this to the transform broadcaster

def turtle_driver():
    rospy.init_node('circle', anonymous=True) #initialise the node as "circle"
    sub = rospy.Subscriber('/turtle1/pose', Pose, pose_callback) #subscribe to turtle1's current pose
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) #publish turtle1's velocity
    rate = rospy.Rate(10) # 10hz

    twist = Twist() #initialise the turtle's movement
    pose = Pose() #initialise the position movement

    while not rospy.is_shutdown(): #while a shutdown call hasn't been made
        #move the turtle in a circle
        twist.linear.x = 1
        twist.angular.z= 0.5
        
        pub.publish(twist) #publish the turtle's movements
        rate.sleep() #allows a ROSInterruptException if shutdown is induced

if __name__ == '__main__':
    try:
        turtle_driver()
    except rospy.ROSInterruptException:
        pass
