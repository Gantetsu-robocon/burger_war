#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import random

class search_mode(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['enemy','flag'])

    def execute(self,userdata):
        #search progarm enemy  or flag
        rospy.loginfo('Executing state search_mode')
        value = random.randint(1,1000)
        print(value)
        if value < 500:
            return 'enemy'
        else:
            return 'flag'

class go_enemy(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['success'])

    def execute(self,userdata):
        rospy.loginfo('Executing state go_enemy')
        rospy.sleep(1)
        return 'success'

    
class go_flag(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['enemy'])

    def execute(self,userdata):
        rospy.loginfo('Executing state go_flag')
        rospy.sleep(1)
        return 'enemy'

    


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
                               transitions={'enemy':'enemy'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()


if __name__ == '__main__':
    main()