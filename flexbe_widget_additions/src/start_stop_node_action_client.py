#! /usr/bin/env python

import roslaunch
import rospy
import numpy as np
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalStatus
from behavior_action_client import BehaviorActionClient

class StartStop:
    def __init__(self):

        self.ros_mode_status = None
        self.change_status_flag = False

        self.ros_mode_sub = rospy.Subscriber("ROSmode", Bool, self.ros_mode_cb)

        # Creates the instance of the client
        self.client = BehaviorActionClient()

        self.load_params()


    def load_params(self):
        self.arg_keys= []
        self.arg_values= []
        self.input_keys=[]
        self.input_values=[]

        # Gets the behavior name
        self.behavior_name = rospy.get_param('~behavior_name', 'Example Behavior')

        try:
            # Gets all the args dictionary
            all_args_dict = rospy.get_param('~args', default={})

            self.arg_keys = all_args_dict.keys()
            temp_arg_values = all_args_dict.values()

            for arg in temp_arg_values:
                self.arg_values.append(str(arg))

            # Get all the inputs dictionary
            all_inputs_dict = rospy.get_param('~inputs', default={})
            self.input_keys = all_inputs_dict.keys()
            temp_input_values = all_inputs_dict.values()

            for arg in temp_input_values:
                self.input_values.append(str(arg))

            rospy.loginfo("Loaded list of arguments.")
        except rospy.ROSException as e:
            rospy.logwarn("Parameter server reports and error: {}".format(e))

        rospy.loginfo("Request '{}' behavior execution.".format(self.behavior_name))

    def ros_mode_cb(self, msg):
        if (msg.data != self.ros_mode_status): # this prevent the restart action of the remote to send some sporadic consecutive True states
            self.ros_mode_status = msg.data
            self.change_status_flag = True

    def main(self):
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            if (self.change_status_flag):
                if (self.ros_mode_status):
                    rospy.loginfo("ROS mode status: {} - START REPEAT".format(self.ros_mode_status))
                    # Sends the goal via action client
                    self.client.send_goal(self.behavior_name, self.arg_keys, self.arg_values, self.input_keys, self.input_values)
                    self.change_status_flag = False
                else:
                    rospy.loginfo("ROS mode status: {} - STOP REPEAT".format(self.ros_mode_status))
                    if (self.client.get_state() == GoalStatus.SUCCEEDED): self.client.stop_tracking_goal()
                    else: self.client.cancel_goal()
                    self.change_status_flag = False
            rate.sleep()


if __name__=="__main__":
    rospy.init_node('start_stop_node')
    ss = StartStop()
    ss.main()
    rospy.spin()