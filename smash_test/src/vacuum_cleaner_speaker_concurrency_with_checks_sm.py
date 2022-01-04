#!/usr/bin/env python
# Author: Gerard Canal <gerard.canal@kcl.ac.uk>
import rospy
import sys
import smach
from smach_ros import IntrospectionServer, SimpleActionState, ServiceState
from move_base_msgs.msg import MoveBaseAction
from vacuum_speaker_wait_smach_states import CheckAndMoveSM, CheckAndTurnSM, MoveUntilObstacleState, CheckChargingState, \
    SpeakerState, SpeakerWaitState
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Bool, Header
from move_base_msgs.msg import MoveBaseGoal


class VacuumCleanerSMNode:
    def __init__(self):
        self.robot_move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def create_sm(self):
        sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
        # Set last spoken here:
        sm.userdata.last_spoken = rospy.Time(0)
        with sm:
            # This will be called if one state has terminated. As I am more interested in waiting for the move action,
            # I am using this. We could also use the outcome_cb, which will be called once ALL children have
            # terminated
            def child_term_cb(outcome_map):
                if outcome_map['MOVING']:  # Stop if MOVING has finished
                    return True
                return False
            cc = smach.Concurrence(outcomes=['obstacle', 'dirt', 'low_battery'], default_outcome='obstacle',
                                   input_keys=['last_spoken'],
                                   output_keys=['last_spoken'],  # Must be in and out, as it is being modified!
                                   child_termination_cb=child_term_cb,
                                   outcome_map={'obstacle': {'MOVING': 'obstacle'},  # This just returns whatever the MOVING state returns
                                                'dirt': {'MOVING': 'dirt'},
                                                'low_battery':  {'MOVING': 'low_battery'}})
            with cc:
                smach.Concurrence.add('SAY_MOVING', SpeakerWaitState('I am now cleaning around'))
                # Version A: using the CheckAndMoveSM
                smach.Concurrence.add('MOVING', CheckAndMoveSM())

                # Option B: Using the MoveUntilObstacleState. Uncomment any, they should be equivalent
                # smach.Concurrence.add('MOVING', MoveUntilObstacleState())
            smach.StateMachine.add('SAY_AND_MOVE', cc, transitions={'obstacle': 'TURNING',
                                                                    'dirt': 'SAY_AND_SPIRAL',
                                                                    'low_battery': 'SAY_AND_CHARGE'})


            smach.StateMachine.add('TURNING', CheckAndTurnSM(),
                                   transitions={'obstacle': 'TURNING',
                                                'no_obstacle': 'SAY_AND_MOVE'})

            # This will be called if one state has terminated. As I am more interested in waiting for the move action,
            # I am using this. We could also use the outcome_cb, which will be called once ALL children have
            # terminated
            def child_term_cb(outcome_map):
                if outcome_map['SPIRALLING']:  # Stop if MOVING has finished
                    return True
                return False
            cc = smach.Concurrence(outcomes=['no_dirt'], default_outcome='no_dirt',
                                   input_keys=['last_spoken'],
                                   output_keys=['last_spoken'],  # Must be in and out, as it is being modified!
                                   child_termination_cb=child_term_cb,
                                   outcome_map={'no_dirt': {'SPIRALLING': 'no_dirt'}})
            with cc:
                smach.Concurrence.add('SAY_SPIRALLING', SpeakerState('I am cleaning this spot'))
                # This could also be a sub state machine hat checks for dirt and spirals
                smach.Concurrence.add('SPIRALLING', SpirallingState())
            smach.StateMachine.add('SAY_AND_SPIRAL', cc, transitions={'no_dirt': 'SAY_AND_MOVE'})

            # This will be called if one state has terminated. As I am more interested in waiting for the move action,
            # I am using this. We could also use the outcome_cb, which will be called once ALL children have
            # terminated
            def child_term_cb(outcome_map):
                if outcome_map['GO_TO_CHARGER']:  # Stop if GO_TO_CHARGER has finished
                    return True
                return False
            cc = smach.Concurrence(outcomes=['succeeded', 'aborted'], default_outcome='succeeded',
                                   input_keys=['last_spoken'],
                                   output_keys=['last_spoken'],  # Must be in and out, as it is being modified!
                                   child_termination_cb=child_term_cb,
                                   outcome_map={'succeeded': {'GO_TO_CHARGER': 'succeeded'},
                                                'aborted': {'GO_TO_CHARGER': 'aborted'}})
            with cc:
                charger_goal = MoveBaseGoal()  # We always go to the same place, so we use a static goal
                charger_goal.target_pose = PoseStamped(Header(frame_id='map'), Pose(Point(7.809, 4.161, 0),
                                                                                    Quaternion(0, 0, 1, 0)))
                smach.Concurrence.add("GO_TO_CHARGER", SimpleActionState('/move_base', MoveBaseAction,
                                                                          goal=charger_goal))

                smach.Concurrence.add('SAY_CHARGING', SpeakerState('My battery is low! Time to recharge'))
            smach.StateMachine.add('SAY_AND_CHARGE', cc, transitions={'succeeded': 'CHARGING',
                                                                      'aborted': 'SAY_AND_MOVE'})


            smach.StateMachine.add("CHARGING", CheckChargingState(), transitions={'charged': 'SAY_AND_MOVE',
                                                                                  'not_charged': 'CHARGING'})
        rospy.sleep(1)  # Sleep for one second to wait for the publishers to register, otherwise the first speech is sometimes lost
        return sm

    def execute_sm(self):
        sm = self.create_sm()
        sis = IntrospectionServer('tea_Server', sm, 'SM_ROOT')
        sis.start()
        outcome = sm.execute()
        sis.stop()
        return outcome


# I added this state here to show you can also put it there. It would be nicer in the imported python file.
class SpirallingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['no_dirt'])
        self.robot_move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.dirt_found = False  # This must be set before the subscriber otherwise it may not work!
        self.dirt_subs = rospy.Subscriber('/dirt', Bool, self.dirt_cb)

    def execute(self, userdata):
        rospy.loginfo('Executing state SPIRALLING')
        d = -1
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.dirt_found:
            d = -d
            for i in range(50):  # Send N messages, arbitrarily
                twist = Twist()
                twist.linear.x = 0.15
                twist.angular.z = d*3
                self.robot_move_pub.publish(twist)
                rate.sleep()
        return 'no_dirt'

    def dirt_cb(self, msg):
        self.dirt_found = msg.data


if __name__ == "__main__":
    rospy.init_node("vacuum_cleaner_sm", sys.argv)
    vacuum = VacuumCleanerSMNode()
    vacuum.execute_sm()
    rospy.spin()
