#!/usr/bin/env python

from flexbe_core import EventState, Logger
from roslib.message import get_message_class
from flexbe_core.proxy import ProxyActionClient

"""
Created on 22/11/2021

@author: Joao Aguizo
"""

class ActionCancelGoal(EventState):
    """
    Generic state to cancel active goals on a ROS action server.

    -- action          string              The action class type name.
    -- topic           string              The action topic name.

    <= goal_cancelled   The goal was cancelled.
    <= inactive         There are not any active goals.
    <= failed           Failed to cancel goal.
    """

    def __init__(self, action="actionlib_tutorials/Fibonacci", topic="fibonacci"):

        super(ActionCancelGoal, self).__init__(outcomes = ['goal_cancelled', 'inactive', 'failed'])

        self._action_topic = topic
        action_class = get_message_class(action + "Action")
        self._client = ProxyActionClient({self._action_topic: action_class})
        self._failed = False
        self._inactive = False


    def execute(self, userdata):
        """Wait for action result and return outcome accordingly"""

        if self._failed:
            return 'failed'
        
        if self._inactive:
            return 'inactive'

        return 'goal_cancelled'


    def on_enter(self, userdata):
        """Try to cancel the active goal (if any)"""

        # Reset the flags
        self._failed = False
        self._inactive = False

        try:
            if self._client.is_active(self._action_topic):

                if not self._client.has_result(self._action_topic):
                    self._client.cancel(self._action_topic)
                    Logger.loginfo("Cancelled '{}' active action goal.".format(self._action_topic))

            else:
                self._inactive = True

        except Exception as e:
            Logger.logwarn("Unable to cancel '{}' goal:\n{}".format(self._action_topic, str(e)))
            self._failed = True
