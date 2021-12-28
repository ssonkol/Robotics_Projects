#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

# from std_msgs.msg import String

def turtle_driver():
    
    topic = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    rospy.init_node('circle_driver', anonymous=True)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        
        twist = Twist()
      
        twist.linear.x = 1
        twist.angular.z= 0.5
      
        topic.publish(twist)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        turtle_driver()
    except rospy.ROSInterruptException:
        pass