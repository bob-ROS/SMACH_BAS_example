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
        # auto generate outcomes
        self.children_ = {key: 'go_to_{}'.format(key) for key in children}
        smach.State.__init__(self, outcomes=list(self.children_.values()), input_keys=['input_request'])

    def execute(self, userdata):
        rospy.sleep(1)
        transition_request = userdata.input_request
        rospy.loginfo('Executing state BAS')
        if transition_request in self.children_:
            return self.children_[transition_request]
        else:
            rospy.logwarn('Attempted to transition to non-existing state, selecting random state')
            return random.choice(list(self.children_.values()))

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

    sm_top.userdata.transition_request = 'MOVEIT'

    # Execute SMACH plan
    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    while not rospy.is_shutdown():
        rospy.sleep(100) # just for blocking node incase of exiting the statemachine


if __name__ == '__main__':
    main()
