#! /usr/bin/env python

import rospy
import actionlib

from flexbe_msgs.msg import BehaviorExecutionAction, BehaviorExecutionGoal, BehaviorExecutionResult

class BehaviorActionClient(object):
    _goal =  BehaviorExecutionGoal()
    _result = BehaviorExecutionResult()

    def __init__(self):
        self.topic_name = 'flexbe/execute_behavior'

        # Creates the simple action client for the behavior execution
        self._client = actionlib.SimpleActionClient(self.topic_name, BehaviorExecutionAction)

        # Announces and waits for the server
        rospy.loginfo("Waiting for '{}' action server...".format(self.topic_name))
        self._client.wait_for_server()
        rospy.loginfo("Ready to send goal requests to '{}'!".format(self.topic_name))


    def send_goal(self, behavior_name, arg_keys= [], arg_values= [], input_keys=[], input_values=[], done_cb= None, active_cb= None, feedback_cb= None):
        
        # Fills the goal 
        self._goal.behavior_name = behavior_name
        self._goal.arg_keys = arg_keys
        self._goal.arg_values = arg_values
        self._goal.input_keys = input_keys
        self._goal.input_values = input_values

        # Sends the goal to the action server.
        self._client.send_goal(self._goal, done_cb, active_cb, feedback_cb)

    
    def wait_for_result(self, timeout=rospy.Duration()):
        self._client.wait_for_result(timeout)

    
    def get_result(self):
        return self._client.get_result()

    
    def get_state(self):
        return self._client.get_state()

    
    def cancel_goal(self):
        self._client.cancel_goal()

    
    def cancel_all_goals(self):
        self._client.cancel_all_goals()

    def stop_tracking_goal(self):
        self._client.stop_tracking_goal()