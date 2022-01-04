# This does not have the shebang (line starting with #!) as this is not meant to be executed and doesn't have a main
# It also means we don't need to give it execution permissions with chmod +x
# Author: Gerard Canal <gerard.canal@kcl.ac.uk>
import rospy
import smach
from smach import CBState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
import random


class CheckAndMoveSM(smach.StateMachine):
    # Note:
    # This could have been done with a single state that checks the obstacles while moving (and probably would have been better).
    # I did it this way to show hierarchical state machines, and to reuse this for the turning, and to show how to use
    # a CBState.
    # The alternative would be a state like CheckObstacleState that has a loop in the execute.
    # The difference would be that the execute would be as follows, and it would only have an outcome called 'obstacle':
    #
    # def execute(self, userdata):
    #     rate = rospy.Rate(10)
    #     while not rospy.is_shutdown():
    #         if self.obstacle_found:
    #             return 'obstacle'
    #         twist = Twist()
    #         # go forward with 70% chance, turn with 30%
    #         forward = random.random() <= 0.6
    #         if forward:
    #             twist.linear.x = random.randint(0, 5)
    #         else:
    #             twist.angular.z = random.randint(0, 5) - 2.5  # this will give values between -2.5 and 2.5
    #         self.robot_move_pub.publish(twist)
    #
    # I have added the state implemented at the end. You can change it in the main state machine to see the differences

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['obstacle', 'dirt', 'low_battery'], input_keys=['last_spoken'],
                                    output_keys=['last_spoken'])  # Must be in and out, as it is being modified!)
        self.robot_move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.dirt_found = False  # This must be set before the subscriber otherwise it may not work!
        self.dirt_subs = rospy.Subscriber('/dirt', Bool, self.dirt_cb)
        self.low_battery = False
        self.battery_subs = rospy.Subscriber('/low_battery', Bool, self.battery_cb)

        with self:
            smach.StateMachine.add('CHECK_FOR_OBSTACLE', CheckObstacleState(),
                                   transitions={'obstacle': 'SAY_OBSTACLE',  # If obstacle, the whole state Machine returns obstacle
                                                'no_obstacle': 'MOVE_THE_ROBOT'},
                                   remapping={'out_speech': 'speech'})

            smach.StateMachine.add('MOVE_THE_ROBOT', CBState(self.move_cb, cb_args=[self]),
                                   transitions={'succeeded': 'CHECK_FOR_OBSTACLE',
                                                'dirt': 'dirt',
                                                'low_battery': 'low_battery'})

            smach.StateMachine.add('SAY_OBSTACLE', SpeakerWaitState(), transitions={'succeeded': 'obstacle'},
                                   remapping={'in_speech': 'speech'})

    # NOTE: The callback decorator (next line) makes this a static member. Therefore, if we want to use the class variables,
    # We need to send self as a variable!!
    @smach.cb_interface(outcomes=['succeeded', 'dirt', 'low_battery'])
    def move_cb(userdata, self):
        twist = Twist()
        # go forward with 70% chance, turn with 30%
        forward = random.random() <= 0.6
        if forward:
            twist.linear.x = random.randint(0, 5)
        else:
            twist.angular.z = random.randint(0, 5)-2.5  # this will give values between -2.5 and 2.5
        self.robot_move_pub.publish(twist)

        if self.dirt_found:
            # Here I am assuming when this state is called again the dirt will not be there anymore.
            # An alternative would be to get this information from userdata
            self.dirt_found = False
            return 'dirt'

        if self.low_battery:
            # Here I am assuming when this state is called again the battery will be charged, so I set it to false for next time
            # An alternative would be to get this information from userdata
            self.low_battery = False
            return 'low_battery'

        # This state can rarely fail...
        return 'succeeded'

    def dirt_cb(self, msg):
        self.dirt_found = msg.data

    def battery_cb(self, msg):
        self.low_battery = msg.data


class CheckAndTurnSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['obstacle', 'no_obstacle'])
        self.robot_move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        with self:
            smach.StateMachine.add('CHOOSE_TURN_DIRECTION', CBState(self.set_direction_cb),
                                   transitions={'succeeded': 'TURN'},
                                   remapping={'out_direction': 'turn_direction'})

            smach.StateMachine.add('TURN', CBState(self.turn_cb, cb_args=[self.robot_move_pub]),
                                   transitions={'succeeded': 'CHECK_FOR_OBSTACLE'},
                                   remapping={'in_direction': 'turn_direction'})

            smach.StateMachine.add('CHECK_FOR_OBSTACLE', CheckObstacleState(),
                                   transitions={'obstacle': 'TURN',  # If obstacle, the whole state Machine returns obstacle
                                                'no_obstacle': 'no_obstacle'})

    # NOTE: The callback decorator (next line) makes this a static member. Therefore, if we want to use the class variables,
    # We need to send what we need as argument. Here, I just send the published
    @smach.cb_interface(outcomes=['succeeded'], input_keys=['in_direction'])
    def turn_cb(userdata, robot_move_pub):
        twist = Twist()
        twist.angular.z = random.randint(0, 5)*userdata.in_direction
        robot_move_pub.publish(twist)
        return 'succeeded'

    # NOTE: The callback decorator (next line) makes this a static member. Therefore, if we want to use the class variables,
    # We need to send self as a variable!! We don't need anything from self here, so we don't send it
    @smach.cb_interface(outcomes=['succeeded'], output_keys=['out_direction'])
    def set_direction_cb(userdata):
        userdata.out_direction = random.choice([-1, 1])
        return 'succeeded'


class CheckObstacleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['obstacle', 'no_obstacle'], output_keys=['out_speech'])
        self.obstacle_threshold = 0.5
        self.obstacle_found = False
        self.scan_subs = rospy.Subscriber('/base_scan', LaserScan, self.scan_cb)

    def scan_cb(self, msg):
        self.obstacle_found = min(msg.ranges) < self.obstacle_threshold

    def execute(self, ud):
        if self.obstacle_found:
            ud.out_speech = 'I found an obstacle!'
            return 'obstacle'
        return 'no_obstacle'


class CheckChargingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['charged', 'not_charged'])
        self.low_battery = False
        self.battery_subs = rospy.Subscriber('/low_battery', Bool, self.battery_cb)

    def execute(self, userdata):
        if self.low_battery:
            rospy.sleep(1)  # Sleep to not spam the CPU, as this will be a self loop.
            # Might be better to implement this in an active loop instead of the self-transition
            return 'not_charged'
        return 'charged'

    def battery_cb(self, msg):
        self.low_battery = msg.data


# Alternative to the CheckAndMoveSM
class MoveUntilObstacleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['obstacle', 'dirt', 'low_battery'])
        self.obstacle_threshold = 0.5
        self.obstacle_found = False
        self.scan_subs = rospy.Subscriber('/base_scan', LaserScan, self.scan_cb)
        self.dirt_found = False  # This must be set before the subscriber otherwise it may not work!
        self.dirt_subs = rospy.Subscriber('/dirt', Bool, self.dirt_cb)
        self.low_battery = False
        self.battery_subs = rospy.Subscriber('/low_battery', Bool, self.battery_cb)
        self.robot_move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def scan_cb(self, msg):
        self.obstacle_found = min(msg.ranges) < self.obstacle_threshold

    def execute(self, userdata):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.obstacle_found:
                return 'obstacle'
            if self.dirt_found:
                # Here I am assuming when this state is called again the dirt will not be there anymore.
                # An alternative would be to get this information from userdata
                self.dirt_found = False
                return 'dirt'

            if self.low_battery:
                # Here I am assuming when this state is called again the battery will be charged, so I set it to false for next time
                # An alternative would be to get this information from userdata
                self.low_battery = False
                return 'low_battery'

            twist = Twist()
            # go forward with 70% chance, turn with 30%
            forward = random.random() <= 0.6
            if forward:
                twist.linear.x = random.randint(0, 5)
            else:
                twist.angular.z = random.randint(0, 5) - 2.5  # this will give values between -2.5 and 2.5
            self.robot_move_pub.publish(twist)
            rate.sleep()

    def dirt_cb(self, msg):
        self.dirt_found = msg.data

    def battery_cb(self, msg):
        self.low_battery = msg.data


# This state can either get the speech from userdata (if no parameter is passed) or use the passed parameter, and it
# will say out loud whatever is passed.
# Note that the version only using the speech parameter (i.e. without setting an input_key would also work well for all
# cases in this exercise, and it is in fact easier to use!
class SpeakerState(smach.State):
    def __init__(self, speech=None):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=[] if speech else ['in_speech'],
                             output_keys=['last_spoken'])
        self.speak_pub = rospy.Publisher('/speech', String, queue_size=1)
        self.speech = speech

    def execute(self, ud):
        if self.speech:
            s = self.speech
        else:
            s = ud.in_speech
        self.speak_pub.publish(s)
        ud.last_spoken = rospy.Time.now()
        # As this published to a topic, it immediately ends. If we do not do anything there, the robot may override the
        # speech and keep interrupting itself. Thus, we'll wait to give it time to finish the speech.
        rospy.sleep(0.05*len(s))  # Wait 0.05 per character in the string. Try to take this line out to see the effect
        return 'succeeded'


# The same state as before, but it will not speak if we have spoken less than 5 seconds ago, to not overstress our user
class SpeakerWaitState(smach.State):
    def __init__(self, speech=None, wait_time=3):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['last_spoken'] if speech else ['last_spoken',
                                                                                                      'in_speech'],
                             output_keys=['last_spoken'])
        self.speak_pub = rospy.Publisher('/speech', String, queue_size=3)
        self.speech = speech
        self.wait_time = wait_time

    def execute(self, ud):
        if (rospy.Time.now()-ud.last_spoken).to_sec() < self.wait_time:
            # We won't speak if we spoken not too long ago.
            # This will only be used for the MOVE and OBSTACLE speaks, which are the ones that tend to overlap themselves
            return 'succeeded'
        ud.last_spoken = rospy.Time.now()
        if self.speech:
            s = self.speech
        else:
            s = ud.in_speech
        self.speak_pub.publish(s)
        # As this published to a topic, it immediately ends. If we do not do anything there, the robot may override the
        # speech and keep interrupting itself. Thus, we'll wait to give it time to finish the speech.
        rospy.sleep(0.05*len(s))  # Wait 0.05 per character in the string. Try to take this line out to see the effect
        return 'succeeded'
