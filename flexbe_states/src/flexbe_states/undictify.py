#!/usr/bin/env python

from flexbe_core import EventState, Logger

"""
Created on 15/02/2022

@author: Joao Aguizo
"""

class Undictify(EventState):
    """
    Extract input_keys in form of string list from dictionary and output them as output_keys to the
    decision making.

    -- output_keys  string[]    The keys to extract from the dictionary.

    ># dictionary   dict        The dictionary to extract info from.

    <= done                     The goal was sent.
    """

    def __init__(self, output_keys=[]):

        super(Undictify, self).__init__(
            outcomes = ['done'],
            input_keys = ['dictionary'],
            output_keys = output_keys
        )

        self._output_keys = output_keys


    def execute(self, _):
        return 'done'


    def on_enter(self, userdata):
        for key in self._output_keys:
            try:
                userdata[key] = userdata.dictionary[key]
            except KeyError as e:
                Logger.logwarn("Failed to get element from dictionary:\n{}".format(str(e)))
                userdata[key] = None # we do this so that the userdata is not kept unset
