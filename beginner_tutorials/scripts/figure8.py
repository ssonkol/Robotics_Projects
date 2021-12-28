import rospy
from geometry_msgs.msg import Twist
import sys

PI = 3.14159265335897

def turtle_circle(radius,speed):
    rospy.init_node('turtlesim', anonymous=True)
    pub = rospy.Publisher('turtle1/cmd_vel',Twist,queue_size=10)
    rate = rospy.Rate(10)
    vel_msg = Twist()

    vel_msg.linear.x = speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = speed/radius

    #Move Robot in circle
    while not rospy.is_shutdown():
        pub.publish(vel_msg)
    
    vel_msg.linear.x = 0
    vel_msg.linear.z = 0
    pub.publish(vel_msg)
    


rospy.loginfo("Radius = %f",radius)
pub.publish(vel)
rate.sleep()

if __name__ == "__main__":
    try:
        turtle_circle(float(7),float(5))
    except rospy.ROSInterruptException:
        pass