#!/usr/bin/env python
import json

from flexbe_core import EventState, Logger


class JsonSerializer(EventState):
	'''
	Example for a state to demonstrate which functionality is available for state implementation.
	This example lets the behavior wait until the given target_time has passed since the behavior has been started.

	-- target_time 	float 	Time which needs to have passed since the behavior started.

	<= continue 			Given time has passed.
	<= failed 				Example for a failure outcome.

	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(JsonSerializer, self).__init__(outcomes = ['done'], input_keys = ['picklable_object'], output_keys = ['json_data'])

		self._json_data = ""


	def execute(self, userdata):
		userdata.json_data = self._json_data
		return 'done'
		

	def on_enter(self, userdata):
		self._json_data = json.dumps(userdata.picklable_object)
