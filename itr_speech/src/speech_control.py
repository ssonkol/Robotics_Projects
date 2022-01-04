#!/usr/bin/env python
# coding=utf-8
# Author: Gerard Canal <gerard.canal@kcl.ac.uk>
import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class SpeechControl:
    def __init__(self):
        self.speech_subs = rospy.Subscriber('/speech_recognition/final_result', String, self.speech_received) #subscribe to the final result of speech recognition
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #initialise the velocity to a publisher
        self.tts_pub = rospy.Publisher('/tts/phrase', String, queue_size=1) #initialise the said phrase to a publisher
        self.twist = Twist() #initialise the robot's movement

    def speech_received(self, msg):
        self.recognized_speech = msg.data #pass the recognised speech to a set variable
        rospy.loginfo("I heard: " + self.recognized_speech) #log what the machine heard
        if 'forward' in self.recognized_speech: #If the word forward is  in the speech
            # move forward
            self.twist.linear.x = 0.5
            self.twist.angular.z = 0.0
            self.tts_pub.publish('I am moving forward')
        elif 'backward' in self.recognized_speech:
            # move backward
            self.twist.linear.x = -0.5
            self.twist.angular.z = 0.0
            self.tts_pub.publish('I am moving backward')
        elif 'left' in self.recognized_speech:
            # move left
            self.twist.angular.z = 1.0
            self.twist.angular.x = 0.0
            self.tts_pub.publish('I am turning left')
        elif 'right' in self.recognized_speech:
            # turn right
            self.twist.angular.z = -1.0
            self.twist.angular.x = 0.0
            self.tts_pub.publish('I am turning right')
        else:
            self.tts_pub.publish('I am stopping')
            self.twist = Twist()

    def main_loop(self):
        rate = rospy.Rate(10) #set this at a rate of 10hz
        while not rospy.is_shutdown(): #while the machine hasn't been shutdown
            self.cmd_pub.publish(self.twist) #publish the robot's movements
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('speech_control', sys.argv) #call the node speech_control
    speech_control = SpeechControl() #start the SpeechControl function
    rospy.loginfo("I am ready")
    speech_control.main_loop()
    rospy.spin()
