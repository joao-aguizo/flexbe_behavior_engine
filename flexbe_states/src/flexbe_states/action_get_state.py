#!/usr/bin/env python

from flexbe_core import EventState, Logger
from roslib.message import get_message_class
from flexbe_core.proxy import ProxyActionClient
from actionlib_msgs.msg import GoalStatus

"""
Created on 12/08/2021

@author: Joao Aguizo
"""


class ActionGetState(EventState):
    """
    Move base flex action state to get result.

    #> result     actionlib_msgs/GoalStatus     The result of the current goal.

    <= goal_result_succeeded    Navigation to target pose succeeded.
    <= goal_result_aborted      Navigation to target pose aborted.
    <= goal_has_feedback        The goal still is active, has feedback.
    <= inactive                 The server is inactive.
    <= failed                   Failed to retrieve result.
    """

    def __init__(self, topic="fibonacci", action="actionlib_tutorials/Fibonacci"):
        """Constructor"""

        super(ActionGetState, self).__init__(outcomes=[
            'PENDING',
            'ACTIVE',
            'PREEMPTED',
            'SUCCEEDED',
            'ABORTED',
            'REJECTED',
            'PREEMPTING',
            'RECALLING',
            'RECALLED',
            'LOST',
            'failed'
        ],
            output_keys=['goal_status'])

        self._action_topic = topic
        action_class = get_message_class(action + "Action")
        self._client = ProxyActionClient({self._action_topic: action_class})
        self._status = GoalStatus.PENDING
        self._failed = False

    def execute(self, userdata):
        """Wait for action result and return outcome accordingly"""

        if self._failed:
            return 'failed'

        if self._status == GoalStatus.PENDING:
            return 'PENDING'
        elif self._status == GoalStatus.ACTIVE:
            return 'ACTIVE'
        elif self._status == GoalStatus.PREEMPTED:
            return 'PREEMPTED'
        elif self._status == GoalStatus.SUCCEEDED:
            return 'SUCCEEDED'
        elif self._status == GoalStatus.ABORTED:
            return 'ABORTED'
        elif self._status == GoalStatus.REJECTED:
            return 'REJECTED'
        elif self._status == GoalStatus.PREEMPTING:
            return 'PREEMPTING'
        elif self._status == GoalStatus.RECALLING:
            return 'RECALLING'
        elif self._status == GoalStatus.RECALLED:
            return 'RECALLED'
        else:  # elif self._status == GoalStatus.LOST:
            return 'LOST'

    def on_enter(self, userdata):
        """Try to get the result"""

        # Reset flags
        self._failed = False

        try:
            self._status = self._client.get_state(self._action_topic)
            userdata.goal_status = self._status

        except Exception as e:
            Logger.logwarn("Unable to retrieve action state:\n%s" % str(e))
            self._failed = True
