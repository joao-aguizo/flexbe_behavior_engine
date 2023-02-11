#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

"""
Created on 22/11/2021

@author: Joao Aguizo
"""

class ActionGetFeedback(EventState):
    """
    Move base flex action state to retrieve the feedback of the running goal.

    #> current_pose     PoseStamped     The current pose of the robot.

    <= goal_has_feedback        Retrieved the feedback.
    <= failed                   Could not retrieve feedback from server.
    """

    def __init__(self, action_class, topic="action", output_keys=[], clear_feedback = True):

        super(ActionGetFeedback, self).__init__(
            outcomes = ['goal_no_feedback', 'goal_has_feedback', 'failed'],
            output_keys = output_keys
        )

        self._action_topic = topic
        self._client = ProxyActionClient({self._action_topic: action_class})
        self._output_keys = output_keys
        self._clear_feedback = clear_feedback
        self._failed = False
        self._has_feedback = False


    def execute(self, userdata):
        """Wait for action result and return outcome accordingly"""

        if self._failed:
            return 'failed'

        if self._has_feedback:
            return 'goal_has_feedback'
        else:
            return 'goal_no_feedback'


    def on_enter(self, userdata):
        """Try to get the feedback of an active goal (if any)"""

        # Reset flags
        self._failed = False
        self._has_feedback = False

        try:

            if self._client.has_feedback(self._action_topic):
                feedback = self._client.get_feedback(self._action_topic)

                # Get feedback attributes based on the output keys
                for key in self._output_keys:
                    try:
                        setattr(userdata, key, getattr(feedback, key))
                    except AttributeError as e:
                        Logger.logwarn("Invalid attempt of get attribute on class:\n{}".format(str(e)))

                self._has_feedback = True

                if self._clear_feedback:
                    self._client.remove_feedback(self._action_topic)
            
            else:
                self._has_feedback = False

        except Exception as e:
            Logger.logwarn("Unable to retrieve '{}' feedback:\n{}".format(self._action_topic, str(e)))
            self._failed = True
