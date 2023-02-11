#!/usr/bin/env python

from flexbe_core import EventState, Logger
from roslib.message import get_message_class
from flexbe_core.proxy import ProxyActionClient


"""
Created on 22/11/2021

@author: Joao Aguizo
"""


class ActionSendGoal(EventState):
    """
    Generic action send goal state.

    -- action_class                 ActionType          The action type class.
    -- action_goal_class            ActionTypeGoal      The action instatiable goal class.  
    -- topic                        string              The action topic name.
    -- input_keys                   string[]            The custom input keys. These should match the action goal properties.
    -- is_pausable                  bool                Is the action pausable on a priority interruption?
    -- cancel_active_goal           bool                Cancel goal if active?

    <= goal_sent                The goal was sent.
    <= goal_active              There is an active goal (only if cancel_active_goal is False)
    <= failed                   Navigation send goal failed.
    """

    def __init__(self, action="actionlib_tutorials/Fibonacci", topic="fibonacci", input_keys=[], is_pausable=True, cancel_active_goal=False):

        super(ActionSendGoal, self).__init__(
            outcomes=['goal_sent', 'goal_active', 'failed'],
            input_keys=input_keys
        )

        self._action_topic = topic
        self._input_keys = input_keys
        self._is_pausable = is_pausable
        self._cancel_active_goal = cancel_active_goal

        action_class = get_message_class(action + "Action")
        self._client = ProxyActionClient({self._action_topic: action_class})
        self._action_goal_class = get_message_class(action + "Goal")

        self._failed = False
        self._goal_active = False
        self._on_pause_cancelled = False
        self._sent_goal = None

    def send_goal(self, goal):
        failed = False
        try:
            self._client.send_goal(self._action_topic, goal)
            self._sent_goal = goal
        except Exception as e:
            Logger.logwarn("Unable to send '{}' action goal:\n{}".format(self._action_topic, str(e)))
            failed = True

        return failed

    def cancel_active_goals(self):
        cancelled = False
        if self._client.is_available(self._action_topic):
            if self._client.is_active(self._action_topic):
                if not self._client.has_result(self._action_topic):
                    self._client.cancel(self._action_topic)
                    cancelled = True
                    Logger.loginfo("Cancelled '{}' active action goal.".format(self._action_topic))

        return cancelled

    def execute(self, userdata):
        """Wait for action result and return outcome accordingly"""

        if self._failed:
            return 'failed'

        if self._goal_active:
            return 'goal_active'

        return 'goal_sent'

    def on_enter(self, userdata):
        """Create and send action goal"""

        # Reset the flags
        self._failed = False
        self._goal_active = False

        if self._client.is_available(self._action_topic):
            if self._client.is_active(self._action_topic):
                if self._cancel_active_goal:
                    self.cancel_active_goals()
                else:
                    self._goal_active = True
                    return

        # Populate action goal
        goal = self._action_goal_class()
        for key in self._input_keys:
            try:
                setattr(goal, key, getattr(userdata, key))
            except AttributeError as e:
                Logger.logwarn("Invalid attempt of set attribute on class:\n{}".format(str(e)))

        # Send the action goal for execution
        self._failed = self.send_goal(goal)

    def on_resume(self, userdata):
        if self._on_pause_cancelled and self._sent_goal:
            self.send_goal(self._sent_goal)  # resend the previous goal

    def on_pause(self, userdata):
        if self._is_pausable:
            self._on_pause_cancelled = self.cancel_active_goals()

    def on_stop(self):
        _ = self.cancel_active_goals()
