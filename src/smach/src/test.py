#!/usr/bin/env python

import roslib
import rospy

from std_msgs.msg import Bool,Int64,String

class Standby():
    def __init__(self):
        rospy.loginfo('Initiating state Standby')

    def execute(self):
        global last_cmd
        rospy.loginfo('Executing state Standby')
        # Wait for "follow me" or "what is that" command
        while True:

            if last_cmd == "follow_people":
                return "continue_follow"
        
            if last_cmd == "what_is_that":
                return "continue_pointing"


def cmd_callback(cmd):
    global last_cmd
    last_cmd = cmd.data

if __name__ == '__main__':
    rospy.init_node('what_is_that_smach')
    last_cmd = ''
    voice_cmd_sub = rospy.Subscriber('/voice_cmd',String,cmd_callback)
    st = Standby()
    print(st.execute())
    
    rospy.spin()