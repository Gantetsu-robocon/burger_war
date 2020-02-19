#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import random

from geometry_msgs.msg import Twist


class search_mode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['enemy','flag'])

    def execute(self):
        #search progarm enemy  or flag
        value = random.randint(1,2)
        if value == 1:
            return 'enemy'
        else:
            return 'flag'

class go_enemy(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['success'])

    def execute(self):
        rospy.loginfo('Executing state go_enemy')
        rospy.sleep(1)
        return 'success'

    
class go_flag(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['success'])

    def execute(self):
        rospy.loginfo('Executing state go_flag')
        rospy.sleep(1)
        return 'success'

    


def main():
    rospy.init_node('smach_example_state_machine')

    sm = smach.StateMachine(outcomes = ['success'])

    with sm:
        # Add states to the container
        smach.StateMachine.add('search_Mode', search_mode(), 
                               transitions={'enemy':'enemy', 
                                            'flag':'flag'})
        smach.StateMachine.add('enemy', go_enemy(), 
                               transitions={'success':'success'})
        smach.StateMachine.add('flag', go_flag(), 
                               transitions={'success':'success'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()


if __name__ == '__main__':
    main()