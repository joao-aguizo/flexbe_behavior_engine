#! /usr/bin/env python

import sys
import rospy
import actionlib
import pickle

from flexbe_msgs.msg import BehaviorInputAction, BehaviorInputResult

class OperatorInputActionServer(object):
    _result = BehaviorInputResult()

    def __init__(self):
        self._as = actionlib.SimpleActionServer('flexbe/operator_input', BehaviorInputAction, execute_cb = self.execute_cb, auto_start = False)
        self._as.start()

        rospy.loginfo('Ready for operator input!')
        
    '''
    Request types:

    ***Any***   >>>   type 'cancel' or any variation to cancel process
    
    0 - Custom question     >>>     [CUSTOM QUESTION] (answer analysis in code)
    1 - Y/N question       >>>     Are you sure you want to '{}'? [Y/n]
    2 - Operator decision state via terminal    >>>     Data contains the possible outcomes, displays and allows the user to choose (0, 1, ..., n)
    '''  
    def execute_cb(self, goal):
        data = ""
        question = ""
        result_code = 0
        outcomes = []
        
        rospy.loginfo("\n\n\n\t OPERATOR INPUT REQUESTED \n\t NOTE: type 'cancel' to abort current behavior \n\n")

        # Question definition (based on predefined types)
        if goal.request_type == 1:
            question = "Are you sure you want to {} [Y/n]? ".format(goal.msg)
        elif goal.request_type == 2:
            question = "Terminal operator decision state reached!" + "\n" + "Possible outcomes:" + "\n"

            outcomes = pickle.loads(goal.msg)
            for idx, out in enumerate(outcomes):
                question = question + "{0} - {1} \n".format(idx, out)

            question = question + "Select index and press [ENTER]: "

        elif goal.request_type == 0:
            question = goal.msg

        # Gathers the user input
        if sys.version_info[0] == 2:
            input_data = raw_input(question)
        else:
            input_data = input(question)

        data = input_data   # this works for the default case of request_type == 0

        # Answer analysis 
        if input_data.lower() == "cancel":
            data = "Canceled upon user request."
            result_code = 2
        else:
            # Analysis for the predefined request types
            if goal.request_type == 1:
                if input_data.lower() in ["", "y", "yes"]:    # pressed ENTER or chosen "yes" (default choice)
                    data = "yes"
                elif input_data.lower() in ["n", "no"]:
                    data = "no"
                else:
                    data = "Invalid option!"
                    result_code = 1

            elif goal.request_type == 2:
                try:
                    idx = int(input_data)
                    if idx > len(outcomes) - 1 or idx < 0:
                        data = "{} is not a valid index".format(idx)
                        result_code = 1
                    else:
                        data = input_data
                except ValueError:
                    data = "Invalid option! Please insert an integer."
                    result_code = 1


        # Python 2: the pickle dumps is mandatory, since the input state executes pickle.loads().
        # Ptyhon 3: pickle.dumps() returns a byte-like object, instead of a string (python 2).
        #           Rospy attempts to encode the message, which leads to an error.
        self._result.result_code = result_code
        if sys.version_info[0] == 2:
            self._result.data = pickle.dumps(data)
        else:
            self._result.data = data

        self._as.set_succeeded(self._result)
        

if __name__ == '__main__':
    rospy.init_node('operator_input_action_server')
    server = OperatorInputActionServer()
    rospy.spin()