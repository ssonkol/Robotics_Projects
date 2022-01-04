#!/usr/bin/env python
# Author: Gerard Canal <gerard.canal@kcl.ac.uk>
import rospy
import sys
import smach
from smach_ros import IntrospectionServer, SimpleActionState, ServiceState
from move_base_msgs.msg import MoveBaseAction
from vacuum_smach_states import CheckAndMoveSM, CheckAndTurnSM, MoveUntilObstacleState, CheckChargingState
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Bool, Header
from move_base_msgs.msg import MoveBaseGoal


class VacuumCleanerSMNode:
    def __init__(self):
        self.robot_move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def create_sm(self):
        sm = smach.StateMachine(outcomes=['succeeded', 'aborted']) # Initialise main SM with two outcomes
        with sm:
            # Version A: using the CheckAndMoveSM
            smach.StateMachine.add('MOVING', CheckAndMoveSM(),
                                   transitions={'obstacle': 'TURNING',#if there is an obstacle keep turning
                                                'dirt': 'SPIRALLING', #if there is dirt start spiraling to clean
                                                'low_battery': 'GO_TO_CHARGER'}) #if there is low battery, go to charger

            # Option B: Using the MoveUntilObstacleState. Uncomment any, they should be equivalent
            # smach.StateMachine.add('MOVING', MoveUntilObstacleState(),
            #                        transitions={'obstacle': 'TURNING',
            #                                     'dirt': 'SPIRALLING',
            #                                     'low_battery': 'GO_TO_CHARGER'})

            smach.StateMachine.add('TURNING', CheckAndTurnSM(), #if the current state is turning, then run the Check and turn function
                                   transitions={'obstacle': 'TURNING', #If there is an obsticle change the state to turning
                                                'no_obstacle': 'MOVING'}) #If there is no obstacle, change the state to moving

            # THis could also be a sub state machine hat checks for dirt and spirals
            smach.StateMachine.add('SPIRALLING', SpirallingState(), #If the current state is Spiralling, then run the Spiralling State function
                                    transitions={'no_dirt': 'MOVING'}) #If there is no longer any dirt, continue moving

            charger_goal = MoveBaseGoal()  # We always go to the same place, so we use a static goal
            charger_goal.target_pose = PoseStamped(Header(frame_id='map'), Pose(Point(7.809, 4.161, 0), #Set the coordinates
                                                                                Quaternion(0, 0, 1, 0))) #set the angle it will be facing

            smach.StateMachine.add("GO_TO_CHARGER", SimpleActionState('/move_base', MoveBaseAction, #If told to go to the charger then move to the goal
                                                                      goal=charger_goal),
                                   transitions={'succeeded': 'CHARGING', 'aborted': 'MOVING', 'preempted': 'MOVING'}) #set all types of transitions

            smach.StateMachine.add("CHARGING", CheckChargingState(),
                                    transitions={'charged': 'MOVING', #once charged, start moving
                                                'not_charged': 'CHARGING'}) #If not charged, continue charging
        return sm

    def execute_sm(self):
        sm = self.create_sm() #create the state machine
        sis = IntrospectionServer('tea_Server', sm, 'SM_ROOT') #instantiate the server for the state machine
        sis.start() #start the server
        outcome = sm.execute() #execute the main cleaner
        sis.stop() #stop the server once execution is complete
        return outcome #return the outcome


# I added this state here to show you can also put it there. It would be nicer in the imported python file.
class SpirallingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['no_dirt'])#only have the outcome no_dirt as it is only ever going to be running to get to that state
        self.robot_move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #set the robot's movements to the publisher
        self.dirt_found = False  # This must be set before the subscriber otherwise it may not work!
        self.dirt_subs = rospy.Subscriber('/dirt', Bool, self.dirt_cb) #subscribe to the dirt topic

    def execute(self, userdata):
        rospy.loginfo('Executing state SPIRALLING') #add to the log that it has started spiralling (cleaning)
        d = -1 #set distance to -1
        rate = rospy.Rate(10) #set the rate to 10hz
        while not rospy.is_shutdown() and self.dirt_found: #if the machine hasn't been shut down and dirt has been found
            d = -d #set the distance to 1
            for i in range(50):  # Send N messages, arbitrarily
                twist = Twist() #initialise the machine's movement
                twist.linear.x = 0.15 #move the machine forward
                twist.angular.z = d*3 #rotate the machine by 3
                self.robot_move_pub.publish(twist) #publish the machine's movements
                rate.sleep()
        return 'no_dirt' #return there is no longer any dirt

    def dirt_cb(self, msg):
        self.dirt_found = msg.data #keep track of the dirt status

if __name__ == "__main__":
    rospy.init_node("vacuum_cleaner_sm", sys.argv) #initialise the node as vacuum_cleaner_sm
    vacuum = VacuumCleanerSMNode() #Initialise the State Machine and all its nodes
    vacuum.execute_sm() #Execute the SM - i.e. start operating
    rospy.spin()
