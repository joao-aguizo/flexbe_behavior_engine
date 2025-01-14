#!/usr/bin/env python

from flexbe_core import EventState, Logger
from roslib.message import get_service_class
from flexbe_core.proxy import ProxyServiceCaller


class ServiceRequestState(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- topic    string  The service topic.
    -- timeout 	float 	Service max wait time.

    <= done 			Executed the service successfully.
    <= command_error	Error executing the service.

    '''

    def __init__(self, service="rospy_tutorials/AddTwoInts", input_keys = [], output_keys = [], topic = "add_two_ints", timeout = 5):
        super(ServiceRequestState, self).__init__(
            outcomes = ['done', 'command_error'],
            input_keys = input_keys,
            output_keys = output_keys 
        )

        service_class = get_service_class(service)
        self._request_class = service_class._request_class

        self._topic_name = topic
        self._input_keys = input_keys
        self._output_keys = output_keys
        self._proxy_caller = ProxyServiceCaller(topics = {self._topic_name : service_class}, wait_duration = timeout)
        self._error = False


    def execute(self, userdata):
        if self._error:
            return 'command_error'

        return 'done'


    def on_enter(self, userdata):
        # Resets error flag
        self._error = False

        try:
            # Populate action goal
            request = self._request_class()
            for key in self._input_keys:
                try:
                    setattr(request, key, getattr(userdata, key))
                except AttributeError as e:
                    Logger.logwarn("Invalid attempt of set attribute on class:\n{}".format(str(e)))

            response = self._proxy_caller.call(self._topic_name, request)

            # Get result attributes based on the output keys
            for key in self._output_keys:
                try:
                    setattr(userdata, key, getattr(response, key))
                except AttributeError as e:
                    Logger.logwarn("Invalid attempt of get attribute on class:\n{}".format(str(e)))

        except ValueError as e:
            Logger.logwarn("Failed to send the '{}' command:\n{}".format(self._topic_name, str(e)))
            self._error = True