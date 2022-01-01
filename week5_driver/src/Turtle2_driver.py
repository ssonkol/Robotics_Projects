#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import tf_conversions
import numpy as np

################# IMPORT all the necessary classes and messages (Twist, Transform message, pose)
threshold = 0.02

def pose_callback(msg):
    global pose # Pose needs to be updated throughout the whole script
    pose = msg #set the current pose to the callbacks recieved pose
    T_2 = TransformStamped() #This will be used to express a transform from coordinate frame to the header/main frame
    tf_broadcaster = tf2_ros.TransformBroadcaster() #set the broadcaster so all turtle's are in sync

    T_2.header.stamp = rospy.Time.now() # This is a transformation between world and turtle 2
    T_2.header.frame_id = "world" #set the common frame i.e. view 1
    T_2.child_frame_id = "turtle2" #set the turtle's fram i.e. view 3 - Turtle 2's perspective
    T_2.transform.translation.x = pose.x #set the current x axis
    T_2.transform.translation.y = pose.y #set the current y axis
    
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, pose.theta) # We use the conversion from Euler to quaternion
    T_2.transform.rotation.x = q[0] #set the x axis
    T_2.transform.rotation.y = q[1] #set the y axis
    T_2.transform.rotation.z = q[2] #set the z axis
    T_2.transform.rotation.w = q[3] #set the w axis - turn angle

    tf_broadcaster.sendTransform(T_2) #send this to the transform broadcaster

def turtle2_driver_tf():

    rospy.init_node('turtle2_driver', anonymous=True) #initialise the node as "turtle2_driver"

    tf_buffer = tf2_ros.Buffer() #allows the tf2_frames to respond to requests
    tf_listener = tf2_ros.TransformListener(tf_buffer) #Listener - the name says it all
    pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10) #publish turtle2's velocity
    sub = rospy.Subscriber('/turtle2/pose', Pose, pose_callback) #subscribe to turtle2's current pose
    rate = rospy.Rate(10) #set the rate to 10hz

    twist = Twist() #initialise the turtle's movement

    while not rospy.is_shutdown(): #while a shutdown call hasn't been made
        try:
            T_12 = tf_buffer.lookup_transform('turtle1', 'turtle2', rospy.Time(0)) #look for the current transform between turtle1 and turtle2

            distance = T_12.transform.translation.x ** 2 + T_12.transform.translation.y ** 2 #calculate the distance between the two
            angle = np.arctan2(T_12.transform.translation.x, T_12.transform.translation.y) #calculate the angle between the two

            if distance > threshold and abs(angle) < threshold: #if turtle2's angle and distance is more than the threshold
                twist.linear.x = 1 #move the turtle
                pub.publish(twist) #publish the turtle's movements

            elif distance < threshold: #if the distance between turtle2 and turtle1 is less than the threshold
                print("Turtle 2 has caught Turtle 1!")

            else: 
                twist.linear.x = 0 #don't move the turtle
        except:
            T_12 = TransformStamped() #reset the transforms of turtle1 and turtle2

        rate.sleep()

if __name__ == '__main__':
    try:
        turtle2_driver_tf()
    except rospy.ROSInterruptException:
        pass