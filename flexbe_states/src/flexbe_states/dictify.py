#!/usr/bin/env python

from genpy import Message
from flexbe_core import EventState
from rospy_message_converter import message_converter

"""
Created on 17/02/2022

@author: Joao Aguizo
"""

class Dictify(EventState):
    """
    Fills in a dictionary with the provided inputs from the input keys.

    -- input_keys               string[]    The keys to input into the dictionary.
    -- binary_array_as_bytes    boolean     If True, ...

    #> dictionary   dict        The dictionary to extract info from.

    <= done                     The goal was sent.
    """

    def __init__(self, input_keys=[], binary_array_as_bytes = True):

        super(Dictify, self).__init__(
            outcomes = ['done'],
            input_keys = input_keys,
            output_keys = ['dictionary']
        )

        self._input_keys = input_keys
        self._binary_array_as_bytes = binary_array_as_bytes


    def execute(self, _):
        return 'done'


    def on_enter(self, userdata):
        value = None
        dictionary = {}
        for key in self._input_keys:
            
            if isinstance(userdata[key], Message):
                value = message_converter.convert_ros_message_to_dictionary(
                    userdata[key],
                    binary_array_as_bytes = self._binary_array_as_bytes
                )
            else:
                value = userdata[key]

            dictionary.update({ key : value })

        userdata.dictionary = dictionary
