#!/usr/bin/env python
import rospy
import sys
from mock_exam.srv import Str2Coord, Str2CoordResponse

# Here I define the coordinates of the rooms. I don't care about orientation in this case.
# The rooms are as follows (imagine below is the map, and the coordinates have the tag next to it)
# _______
# |A B C|
# |D E F|
#
# ROOMS = {'library': [1.74434804916, 8.51225471497],   # A
#          'dining': [5.90392875671, 8.75152206421],    # B
#          'living': [10.5052347183, 8.23617553711],    # C
#          'bedroom': [1.79956388474, 2.71460771561],   # D
#          'entrance': [5.92233419418, 3.02749752998],  # E
#          'kitchen': [10.7260971069, 2.16245126724]}   # F

class StrToCoordNode:
    def __init__(self):
        self.coord_srv = rospy.Service('/str_to_coord', Str2Coord, self.service_cb)

        self.coordinates = {'kitchen': [10.7309074402, 2.15360951424], #F
                            'living': [10.4872121811, 8.59600639343],#c
                            'entrance': [5.92233419418, 3.02749752998],  # E
                            'dining': [5.90392875671, 8.75152206421],    # B
                            'rest': [10.7, 5.13]}

    def service_cb(self, req):
        message_data = req.place
        response = Str2CoordResponse()
        words = message_data.split(' ')
        for word in words:
            if word in self.coordinates:
                response.coordinates.header.frame_id = "map"
                response.coordinates.header.stamp = rospy.Time.now()
                response.coordinates.pose.position.x = self.coordinates[word][0]
                response.coordinates.pose.position.y = self.coordinates[word][1]
                response.coordinates.pose.orientation.w = 1
                return response
        rospy.logerr("I don't know where that location is. Please try again!")
        


if __name__ == "__main__":
    rospy.init_node("string_to_coord_srv", sys.argv)
    str2coord = StrToCoordNode()
    rospy.loginfo('I am running')
    rospy.spin()