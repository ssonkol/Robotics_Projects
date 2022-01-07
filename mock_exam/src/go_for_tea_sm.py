#!/usr/bin/env python
import rospy
import sys
import smach
from smach_ros import IntrospectionServer, SimpleActionState, ServiceState
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction
from mock_exam.srv import Str2Coord, Str2CoordRequest, YOLOLastFrame
#from itr_pkg.srv import YOLOLastFrame  # Change this to your package name


class WaitForRequest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['request_received', 'not_request_received'], output_keys=['in_request_loc'])
        self.request = []
        #self.request_subs = rospy.Subscriber('/get_me_tea', String, self.request_cb)
        self.request_subs = rospy.Subscriber('/speech_recognition/final_result', String, self.request_cb)

    def request_cb(self, msg):
        self.request.append(msg.data)

    def execute(self, userdata):
        rospy.loginfo('Executing state WAITFORREQUEST with queue: ' + str(self.request))
        if self.request:
            userdata.in_request_loc = self.request.pop(0)
            return 'request_received'
        else:
            rospy.sleep(0.5)
            return 'not_request_received'


class RobotSMNode:
    def __init__(self):
        pass

    def create_sm(self):
        sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
        with sm:
            smach.StateMachine.add('WAIT_FOR_REQUEST', WaitForRequest(),
                                   transitions={'request_received': 'GET_REQUEST_COORDS',
                                                'not_request_received': 'WAIT_FOR_REQUEST'},
                                   remapping={'in_request_loc': 'request_loc'})

            smach.StateMachine.add('GET_REQUEST_COORDS', ServiceState('/str_to_coord', Str2Coord,
                                                                      request_slots=['place'],
                                                                      response_slots=['coordinates']),
                                   transitions={'succeeded': 'GO_TO_REQUEST', 'preempted': 'aborted'},
                                   remapping={'coordinates': 'request_coord',
                                              'place': 'request_loc'})

            smach.StateMachine.add("GO_TO_REQUEST", SimpleActionState('/move_base', MoveBaseAction,
                                                                      goal_slots=['target_pose']),
                                   transitions={'succeeded': 'FIND_OBJECT_REQUEST', 'aborted': 'WAIT_FOR_REQUEST', 'preempted': 'WAIT_FOR_REQUEST'},
                                   remapping={'target_pose': 'request_coord'})

            # Request is empty, so we do not need to do anything for the request
            smach.StateMachine.add('FIND_OBJECT_REQUEST', ServiceState('/detect_frame', YOLOLastFrame,
                                                                    response_slots=['detections']),
                                   transitions={'succeeded': 'SPEAK',
                                                'preempted': 'FIND_OBJECT_REQUEST',
                                                'aborted': 'WAIT_FOR_REQUEST'})

            # If I do not do any remapping, the same variable can be used in the userdata. however, if we don't declare
            # it as both input and output key, we won't be able to write on that value.
            # If we see a bottle, we will end the SM (and the program)
            smach.StateMachine.add('SPEAK', DictateRobotState(), transitions={'succeeded': 'WAIT_FOR_REQUEST',
                                                                                'stop': 'succeeded'})
        return sm

    def execute_sm(self):
        sm = self.create_sm()
        sis = IntrospectionServer('Parrot_Server', sm, 'SM_ROOT')
        sis.start()
        outcome = sm.execute()
        sis.stop()
        return outcome

class DictateRobotState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'stop'], input_keys=['detections'])
        self.tts_pub = rospy.Publisher('/tts/phrase', String, queue_size=1)  # This one uses speech_database

    def execute(self, ud):  # ud is the userdata
        for detection in ud.detections:
            rospy.loginfo("I saw a " + detection.name + " with a confidence of " + str(detection.confidence))
            self.tts_pub.publish("I saw a " + detection.name + " with a confidence of " + str(detection.confidence))

        return 'succeeded'

if __name__ == "__main__":
    rospy.init_node("go_for_tea_sm", sys.argv)
    get_Tea = RobotSMNode()
    outcome = get_Tea.execute_sm()
    rospy.loginfo('I have completed execution with outcome: ' + outcome)
    rospy.spin()
