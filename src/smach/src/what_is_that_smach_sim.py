#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from nav_msgs.msg import Odometry
from math import pi
import tf
import tf2_msgs
import tf2_ros

from geometry_msgs.msg import PoseStamped, Twist ,Vector3, TransformStamped
from std_msgs.msg import Bool,Int64,String

# Brings in the SimpleActionClient
import actionlib

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
import time



class Standby(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_follow','continue_pointing'])
        rospy.loginfo('Initiating state Standby')

    def execute(self, userdata):
        global last_cmd
        rospy.loginfo('Executing state Standby')
        # Wait for "follow me" or "what is that" command
        while True:

            if last_cmd == "follow_people":
                return "continue_follow"
        
            if last_cmd == "what_is_that":
                return "continue_pointing"

            time.sleep(0.01)


        
class Stop_command(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_stop','continue_find_person'])
        rospy.loginfo('Initiating state Stop_command')


    def execute(self, userdata):
        rospy.loginfo('Executing state Stop_command')
        global last_cmd
        global target_lost
        global is_stop
        # Wait for "stop" command or target lost
        while True:
            if last_cmd == "stop":
                is_stop = True
                
                return "continue_stop"

            if target_lost:
                return "continue_find_person"

            time.sleep(0.01)


class Follow_person(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_stop','continue_find_person'])

        rospy.loginfo('Initiating state Follow_person')
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.cmd_pub = rospy.Publisher("/walkie/cmd_vel",Twist,queue_size=1)

        rospy.sleep(1)
        rospy.loginfo("Waiting for Action Server")
        self.client.wait_for_server()
        rospy.loginfo("Action server is up, we can send new goal!")

    def execute(self, userdata):
        rospy.loginfo('Executing state Follow_person')
        global target_lost
        global is_stop
        pose = TransformStamped()
        while True:
            
            try:
                pose = self.tfBuffer.lookup_transform('base_footprint','human_base_footprint',rospy.Time.now()-rospy.Duration.from_sec(0.5))

                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "base_footprint"
                goal.target_pose.header.stamp = rospy.Time.now()-rospy.Duration.from_sec(0.5)
                goal.target_pose.pose.position.x = pose.transform.translation.x
                goal.target_pose.pose.position.y = pose.transform.translation.y
                goal.target_pose.pose.orientation = pose.transform.rotation

                rospy.loginfo("Sending new goal: Quarternion is {}, {}, {}, {}".format(pose.transform.rotation.w,pose.transform.rotation.x,pose.transform.rotation.y,pose.transform.rotation.z))

                self.client.send_goal(goal)

                if  is_stop:
                    self.wait = self.client.cancel_goal()

                    cancel = Twist()
                    stop_pub = rospy.Publisher("cmd_vel",Twist,queue_size=1)

                    cancel.linear.x = 0.0
                    cancel.linear.y = 0.0
                    cancel.angular.z = 0.0
                    
                    stop_pub.publish(cancel)

                    return "continue_stop"

                if target_lost:
                    # wait untill the robot reaches the lastest goal
                    wait = self.client.wait_for_result()
                    # If the robot reaches the lastest goal
                    if wait:
                        return "continue_find_person"
                else:
                    # pass
                    rospy.loginfo("Waiting for result")
                    wait = self.client.wait_for_result(rospy.Duration.from_sec(1.0))
                # rospy.sleep(1)
            except Exception as e:
                pass
                

class Get_bounding_box(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_stop','continue_find_person'])
        rospy.loginfo('Initiating state Get_bounding_box')

    def execute(self, userdata):
        rospy.loginfo('Executing state Get_bounding_box')
        global target_lost
        global is_stop        
        

        while True:
            if  target_lost:
                return 'continue_find_person'
            
            elif is_stop:
                return 'continue_stop'


class Rotate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_found_person','continue_stop_rotate'])
        rospy.loginfo('Initiating state Rotating')
        self.move = Twist()

    def execute(self, userdata):
        global target_lost, stop_rotate
        rospy.loginfo('Executing state Rotate')

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # set all value to 0
        self.move.linear.x = 0.0
        self.move.linear.y = 0.0
        self.move.angular.z = 0
        self.pub.publish(self.move)

        rospy.sleep(2)

        duration = 20

        #set angular velocity to pi*4/duration rad/s
        self.move.angular.z = pi*4/duration
        self.pub.publish(self.move)

        start = time.time()
        while time.time() - start < duration:
            if not target_lost:
                self.move.angular.z = 0
                self.pub.publish(self.move)
                return 'continue_found_person'

        stop_rotate = True
        return 'continue_stop_rotate'

            
class Check_bounding_box(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_found_person','continue_stop_rotate'])
        rospy.loginfo('Initiating state Check_bounding_box')   
               
    def execute(self, userdata):
        global target_lost, stop_rotate
        rospy.loginfo('Executing state Check_bounding_box')
        while True:
            if  target_lost == False:
                return 'continue_found_person'
            elif stop_rotate == True:
                return 'continue_stop_rotate'


class Pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_text_to_speech'])
        rospy.loginfo('Initiate state Pose')
            
    def execute(self, userdata):
        rospy.loginfo('Executing state Pose')
        
        return 'continue_text_to_speech'


class Text_to_speech(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_succeeded'])
    def execute(self, userdata):
        rospy.loginfo('Executing state Text_to_speech')
        return 'continue_succeeded'

def cmd_callback(cmd):
    global last_cmd
    last_cmd = cmd.data

def target_lost_callback(tl):
    global target_lost
    target_lost = tl.data

if __name__ == '__main__':
    rospy.init_node('what_is_that_smach')

    target_lost = False
    person_id = None
    is_stop = False
    stop_rotate = False
    last_cmd = ''
    target_lost = False
    voice_cmd_sub = rospy.Subscriber('/voice_cmd',String,cmd_callback)
    target_lost_sub = rospy.Subscriber('target_lost',Bool,target_lost_callback)
    
    

    # Start state machine
    sm = smach.StateMachine(outcomes=['Succeeded','Aborted'])
    with sm:
        smach.StateMachine.add('Standby',Standby(),
                        transitions={'continue_follow':'FOLLOW','continue_pointing':'OBJECT_DETECTION'})
        
        # Create sub smach state machine "FOLLOW"
        sm_follow = smach.Concurrence(outcomes=['Stop','Find_person'],
                                        default_outcome = 'Stop',
                                        outcome_map = {'Find_person':{'Stop_command':'continue_find_person','Follow_person':'continue_find_person','Get_bounding_box':'continue_find_person'}})
        with sm_follow:
            smach.Concurrence.add('Stop_command',Stop_command())
            smach.Concurrence.add('Follow_person',Follow_person())
            smach.Concurrence.add('Get_bounding_box',Get_bounding_box())
        smach.StateMachine.add('FOLLOW', sm_follow, transitions={'Find_person':'FIND_PERSON','Stop':'Aborted'})
        
        # Create sub smach state machine "FIND_PERSON"
        sm_find_person = smach.Concurrence(outcomes=['Stop_rotate','Found_person'],
                                            default_outcome='Stop_rotate',
                                            outcome_map = {'Found_person':{'Check_bounding_box':'continue_found_person'}})
        with sm_find_person:
             smach.Concurrence.add('Rotate',Rotate())
             smach.Concurrence.add('Check_bounding_box',Check_bounding_box())
        smach.StateMachine.add('FIND_PERSON',sm_find_person, transitions={'Stop_rotate':'Aborted','Found_person':'FOLLOW'})
        
        # Create sub smach state machine "OBJECT_DETECTION"
        sm_object_detection = smach.StateMachine(outcomes=['Succeeded'])
        with sm_object_detection:
            smach.StateMachine.add('Pose',Pose(),transitions={'continue_text_to_speech':'Text_to_speech'}) 
            smach.StateMachine.add('Text_to_speech',Text_to_speech(),transitions={'continue_succeeded':'Succeeded'})
        smach.StateMachine.add('OBJECT_DETECTION',sm_object_detection,transitions={'Succeeded':'Standby'})
        
        # Set up
        sis = smach_ros.IntrospectionServer('Server_name',sm,'/PatterRoot')
        sis.start()
        outcome = sm.execute()
        rospy.spin()
        sis.stop()