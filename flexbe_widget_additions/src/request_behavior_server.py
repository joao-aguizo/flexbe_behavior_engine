#! /usr/bin/env python

import rospy
import rosparam
import actionlib
from threading import Lock
from flexbe_msgs.srv import RequestBehavior, RequestBehaviorResponse
from flexbe_msgs.msg import BehaviorExecutionAction, BehaviorExecutionGoal


class RequestBehaviorServer:
    _goal = BehaviorExecutionGoal()

    def __init__(self):
        # Creates the action server
        self.server = rospy.Service('flexbe/request_behavior', RequestBehavior, self.handle_request)
        self.client = actionlib.SimpleActionClient('flexbe/execute_behavior', BehaviorExecutionAction)

        # Data lock
        self.data_lock = Lock()

        # Timer keep
        self._timer = None

        self.client.wait_for_server()

        rospy.loginfo("Ready for requests!")


    def timer_triggered_cb(self, _):
        self.client.send_goal(self._goal)
        self.client.stop_tracking_goal()
        
        # Log the trigger of the timer
        rospy.loginfo("Sent '{}' behavior request with (arg_keys, arg_values, input_keys, input_values): {}"
        .format(self._goal.behavior_name, (self._goal.arg_keys, self._goal.arg_values, self._goal.input_keys, self._goal.input_values)))


    def handle_request(self, req):
        self.data_lock.acquire()

        # Stops the previous timer from triggering (if any)
        if self._timer:
            self._timer.shutdown()

        try:
            rospy.loginfo("Attempting load from YAML...")
            if not req.yaml_path:
                raise Exception("Provided YAML path is empty, ignoring...")

            param_dict = rosparam.load_file(req.yaml_path)[0][0]
            self._goal.behavior_name = param_dict["behavior_name"]
            self._goal.arg_keys = param_dict["args"].keys()
            self._goal.arg_values = [str(v) for v in param_dict["args"].values()]

            # KISS
            self._goal.input_keys = req.input_keys
            self._goal.input_values = req.input_values

            rospy.loginfo("Load from YAML succeeded.")

        except Exception as e:
            rospy.loginfo("YAML load failed. Loading normally... {}".format(e))
            self._goal.behavior_name = req.behavior_name
            self._goal.arg_keys = req.arg_keys
            self._goal.arg_values = req.arg_values
            self._goal.input_keys = req.input_keys
            self._goal.input_values = req.input_values

        # Set the timer to trigger the new behavior: if the integer is lower than 1, 
        # we force to 1 to avoid the crash the timer thread with a division by 0 exception.
        timeout_secs = req.timeout_secs
        if timeout_secs < 1:
            timeout_secs = 1

        rospy.loginfo("Timer set to {}s before triggering request new behavior rotation.".format(timeout_secs))
        self._timer = rospy.Timer(rospy.Duration(timeout_secs), self.timer_triggered_cb, oneshot=True)

        self.data_lock.release()

        return RequestBehaviorResponse("Scheduled behavior request '{}' in {}s".format(str(req.behavior_name), str(timeout_secs)))



if __name__ == '__main__':
    rospy.init_node('request_behavior_server')
    server = RequestBehaviorServer()
    rospy.spin()
