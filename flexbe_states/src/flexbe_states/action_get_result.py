#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

"""
Created on 22/11/2021

@author: Joao Aguizo
"""

class ActionGetResult(EventState):
    """
    Move base flex action state to retrieve the result of the running goal.

    #> current_pose     PoseStamped     The current pose of the robot.

    <= goal_has_feedback        Retrieved the feedback.
    <= failed                   Could not retrieve feedback from server.
    """

    def __init__(self, action_class, topic="action", output_keys = [], clear_result = True):

        super(ActionGetResult, self).__init__(
            outcomes = ['goal_no_result', 'goal_has_result', 'failed'],
            output_keys = output_keys
        )

        self._action_topic = topic
        self._output_keys = output_keys
        self._client = ProxyActionClient({self._action_topic: action_class})
        self._clear_result = clear_result
        self._failed = False
        self._has_result = False


    def execute(self, userdata):
        """Wait for action result and return outcome accordingly"""

        if self._failed:
            return 'failed'

        if self._has_result:
            return 'goal_has_result'
        else:
            return 'goal_no_result'


    def on_enter(self, userdata):
        """Try to get the feedback of an active goal (if any)"""

        # Reset flags
        self._failed = False
        self._has_result = False

        try:

            if self._client.has_result(self._action_topic):
                result = self._client.get_result(self._action_topic)

                # Get result attributes based on the output keys
                for key in self._output_keys:
                    try:
                        setattr(userdata, key, getattr(result, key))
                    except AttributeError as e:
                        Logger.logwarn("Invalid attempt of get attribute on class:\n{}".format(str(e)))

                self._has_result = True

                if self._clear_result:
                    self._client.remove_result(self._action_topic)
            
            else:
                self._has_result = False

        except Exception as e:
            Logger.logwarn("Unable to retrieve navigation result:\n%s" % str(e))
            self._failed = True
