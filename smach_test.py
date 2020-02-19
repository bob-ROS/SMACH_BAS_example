#!/usr/bin/env python

import roslib;

import rospy
import smach
import smach_ros
import random

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'], output_keys=['output_request'])
        self.counter = 0

    def execute(self, userdata):
        rospy.sleep(0.3)
        rospy.loginfo('Executing state FOO')
        userdata.output_request = 'none-existing-state'
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            self.counter = 0
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.sleep(0.3)
        rospy.loginfo('Executing state BAR')
        return 'outcome1'


# define state Bas
class Bas(smach.State):
    def __init__(self, children):
        self.children_ = children.copy()
        #del self.children_['BAS']
        outcomes = list()
        for key in self.children_:  # auto generate outcomes
            outcomes.append('go_to_{}'.format(key))
        self.children_.pop('BAS', None) # BAS is not something we wish to transition to. The goto_BAS transition still exists because you have to also solve this at the other end
        smach.State.__init__(self, outcomes=outcomes, input_keys=['input_request'])

    def execute(self, userdata):
        rospy.sleep(1)
        transition_request = userdata.input_request
        rospy.loginfo('Executing state BAS')
        if transition_request in self.children_:
            return 'go_to_{}'.format(transition_request)
        else:
            rospy.logerr('Attempted to transition to non-existing state, selecting random state')
            return 'go_to_{}'.format(random.choice(list(self.children_.keys())))

        return 'go_to_MOVE_BASE'


def main():
    rospy.init_node('smach_example_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['Exit'])

    # Open the container
    with sm_top:
        # Create the sub SMACH state machine
        sm_sub = smach.StateMachine(outcomes=['outcome4'], output_keys=['transition_request'])
        sm_top.set_initial_state(['BAS'])

        # Open the container
        with sm_sub:
            # Add states to the container
            smach.StateMachine.add('FOO', Foo(),
                                   transitions={'outcome1': 'BAR',
                                                'outcome2': 'outcome4'},
                                   remapping={'output_request': 'transition_request'})
            smach.StateMachine.add('BAR', Bar(),
                                   transitions={'outcome1': 'FOO'})

        smach.StateMachine.add('MOVE_BASE', sm_sub,
                               transitions={'outcome4': 'BAS'})
        smach.StateMachine.add('ANOTHER_MOVE_BASE', sm_sub,
                               transitions={'outcome4': 'BAS'})
        smach.StateMachine.add('WHATEVER', sm_sub,
                               transitions={'outcome4': 'BAS'})
        smach.StateMachine.add('MOVEIT', sm_sub,
                               transitions={'outcome4': 'BAS'})
        smach.StateMachine.add('TRASH', sm_sub,
                               transitions={'outcome4': 'BAS'})
        smach.StateMachine.add('SOMETHING_ELSE', sm_sub,
                               transitions={'outcome4': 'BAS'})
        smach.StateMachine.add('MATLAB', sm_sub,
                               transitions={'outcome4': 'BAS'})
        children = sm_top.get_children()
        transitions = dict()
        for key in children:
            transitions['go_to_{}'.format(key)] = key
        smach.StateMachine.add('BAS', Bas(children),
                               transitions=transitions,
                               remapping={'input_request': 'transition_request'})
    #sm.userdata.sm_counter = 0
    #sm_top.userdata.input_request = 0
    sm_top.userdata.transition_request = 'MOVEIT'

    # Execute SMACH plan
    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    while not rospy.is_shutdown():
        rospy.sleep(100)    # just for blocking node


if __name__ == '__main__':
    main()