#!/usr/bin/env python
# coding=utf-8
# Author: Gerard Canal <gerard.canal@kcl.ac.uk>
import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# Here I define the coordinates of the rooms. I don't care about orientation in this case.
# The rooms are as follows (imagine below is the map, and the coordinates have the tag next to it)
# _______
# |A B C|
# |D E F|
# ¨¨¨¨¨¨¨
ROOMS = {'library': [1.74434804916, 8.51225471497],   # A
         'dining': [5.90392875671, 8.75152206421],    # B
         'living': [10.5052347183, 8.23617553711],    # C
         'bedroom': [1.79956388474, 2.71460771561],   # D
         'entrance': [5.92233419418, 3.02749752998],  # E
         'kitchen': [10.7260971069, 2.16245126724]}   # F

#note change locations 
DRINKS = {'tea': [6.5,9.5],   # A
         'coffee': [6.6, 1.55],    # B
         'chocolate': [11.6, 1.97],    # C
         'donut': [5.88, 1.91],   # D
         'pretzel': [1.63, 1.29],  # E
         'water': [12.3, 5.36]}   # F

class SpeechControlAndRoomNavigation:
    def __init__(self):
        self.speech_subs = rospy.Subscriber('/speech_recognition/final_result', String, self.speech_received) #subscribe to the final result of speech recognition
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #initialise the velocity to a publisher
        self.tts_pub = rospy.Publisher('/speech', String, queue_size=1)  # This one uses speech_database
        self.twist = Twist() #initialise the robot's movement
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.place = ''

    def speech_received(self, msg):
        self.recognized_speech = msg.data
        rospy.loginfo("I heard: " + self.recognized_speech)
        
        # elif 'right' in self.recognized_speech:
        #     if self.move_base_client.get_state() == actionlib.GoalStatus.ACTIVE:
        #         self.tts_pub.publish('I am already going to the ' + self.place + ', tell me to stop to control me.')
        #     else:
        #         # turn right
        #         self.twist.angular.z = -1.0
        #         self.twist.angular.x = 0.0
        #         self.tts_pub.publish('I am turning right')
        # elif 'stop' in self.recognized_speech:
        #     self.move_base_client.cancel_goal()
        #     self.tts_pub.publish('I am stopping')
        #     self.twist = Twist()

        #Get the drink first
            #Look for drink key words --> find them --> store all of them in a list
            #Loop through list
                #Get all drinks at once then deliver to the common location
        #Get the location of delivery and go there
        if self.recognized_speech != None:
            drinks_list = []
            coordinates_list = []
            # Here I will split the recognized sentence, and look if any word matches our list of rooms. It may not be
            # the most efficient approach though.
            words = self.recognized_speech.split(' ')
            for w in words:
                if w in DRINKS:
                    #store to list
                    drinks_list.append(w)
                    continue
                elif w in ROOMS:
                    #option 1
                    #coordinates_list.append(w)
                    #option 2
                    room_name = w
                    continue
                else:
                    rospy.loginfo("Sorry I couldn't find any of those drinks or rooms, please try again.")

            #Let's now collect all of the drinks
            for drink in drinks_list:
                coordinates = DRINKS[drink]
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.pose.position.x = coordinates[0]
                goal.target_pose.pose.position.y = coordinates[1]
                goal.target_pose.pose.orientation.w = 1  # I don't care about the orientation here
                self.move_base_client.send_goal(goal)
                self.tts_pub.publish('I am going to get the ' + drink)
                self.move_base_client.wait_for_result()
                #keep note - it might change to the final drinks list as it overwrites locations - might need to fix
                #break #check if break statement exits for loop or use pass
            
            #Robot goes to the delivery location
            #option 1
            # for coordinate in coordinates_list:
            #     coordinates = ROOMS[coordinate]
            #     goal = MoveBaseGoal()
            #     goal.target_pose.header.frame_id = 'map'
            #     goal.target_pose.pose.position.x = coordinates[0]
            #     goal.target_pose.pose.position.y = coordinates[1]
            #     goal.target_pose.pose.orientation.w = 1  # I don't care about the orientation here
            #     self.move_base_client.send_goal(goal)
            #     self.tts_pub.publish('I am going to the delivery location:' + coordinate)
            #     #keep note - it might change to the final drinks list as it overwrites locations - might need to fix
            #     #break #check if break statement exits for loop or use pass

            #option 2 - there is only one delivery location
            coordinates = ROOMS[room_name]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = coordinates[0]
            goal.target_pose.pose.position.y = coordinates[1]
            goal.target_pose.pose.orientation.w = 1  # I don't care about the orientation here
            self.move_base_client.send_goal(goal)
            self.tts_pub.publish('I am going to the delivery location: ' + coordinates)
            self.move_base_client.wait_for_result()

            #Send the robot back to its home base
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = 8.86
            goal.target_pose.pose.position.y = 0.65
            goal.target_pose.pose.orientation.w = 1  # I don't care about the orientation here
            self.move_base_client.send_goal(goal)
            self.tts_pub.publish('I am going back to my request pod')
            self.move_base_client.wait_for_result()

    def main_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.move_base_client.get_state() != actionlib.GoalStatus.ACTIVE:
                self.cmd_pub.publish(self.twist)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('speech_control', sys.argv)
    speech_control = SpeechControlAndRoomNavigation()
    rospy.loginfo("I am ready")
    rospy.loginfo("Please give a drink that you want and where I should deliver it too")
    speech_control.main_loop()
    rospy.spin()

#test cases
#What if:
#bring me drink x,y,z to location p (Start)

#bring me drink x, y ,z  and deliver x to p but deliver y,z to q (End)