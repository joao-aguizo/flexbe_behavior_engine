#! /usr/bin/env python

import rospy
from behavior_action_client import BehaviorActionClient


if __name__ == '__main__':
    rospy.init_node('flexbe_action_client')

    # Creates the instance of the client
    client = BehaviorActionClient()

    # Gets the behavior name
    behavior_name = rospy.get_param('~behavior_name', 'Example Behavior')

    arg_keys= []
    arg_values= []
    input_keys=[]
    input_values=[]
    try:
        # Gets all the args dictionary
        all_args_dict = rospy.get_param('~args', default={})

        arg_keys = all_args_dict.keys()
        temp_arg_values = all_args_dict.values()

        for arg in temp_arg_values:
            arg_values.append(str(arg))

        # Get all the inputs dictionary
        all_inputs_dict = rospy.get_param('~inputs', default={})
        input_keys = all_inputs_dict.keys()
        temp_input_values = all_inputs_dict.values()

        for arg in temp_input_values:
            input_values.append(str(arg))

        # rospy.logwarn(arg_keys)
        # rospy.logwarn(temp_arg_values)
        # rospy.logwarn(arg_values)
        # rospy.logwarn(input_keys)
        # rospy.logwarn(input_values)

        rospy.loginfo("Loaded list of arguments.")
    except rospy.ROSException as e:
        rospy.logwarn("Parameter server reports and error: {}".format(e))

    rospy.loginfo("Request '{}' behavior execution.".format(behavior_name))

    # Sends the goal via action client
    client.send_goal(behavior_name, arg_keys, arg_values, input_keys, input_values)

    # Logs waiting for result...
    rospy.loginfo("Waiting for '{}' server result...".format(client.topic_name))

    # Waits for result via action client
    client.wait_for_result()

    # Logs result in terminal
    rospy.loginfo("Finish behavior '{}' execution with result: \n[{}]".format(behavior_name, client.get_result()))
    
    # rospy.spin()
    exit()