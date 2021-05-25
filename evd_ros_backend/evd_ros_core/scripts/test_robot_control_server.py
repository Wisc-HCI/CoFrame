#!/usr/bin/env python3

'''
GUI provides play controls for robot server and presents primitive feedback.
'''

import json
import rospy
import tkinter as tk

from evd_interfaces.robot_control_interface import RobotControlInterface


class TestRobotControlServerNode:

    def __init__(self):

        self.var_error_msg = tk.StringVar()
        self.var_tokens_msg = tk.StringVar()
        self.var_status_uuid = tk.StringVar()
        self.var_status_start_time = tk.StringVar()
        self.var_status_prev_time = tk.StringVar()
        self.var_status_curr_time = tk.StringVar()
        self.var_status_stop_time = tk.StringVar()

        self._root = tk.Tk()
        controlFrame = tk.LabelFrame(self._root, text="Controls")
        controlFrame.pack()

        self.play_button = tk.Button(controlFrame, text="Play", command=self._on_play_cb)
        self.play_button.pack()
        self.stop_button = tk.Button(controlFrame, text="Stop", command=self._on_stop_cb)
        self.stop_button.pack()
        self.pause_button = tk.Button(controlFrame, text="Pause", command=self._on_pause_cb)
        self.pause_button.pack()
        self.reset_button = tk.Button(controlFrame, text="Reset", command=self._on_reset_cb)
        self.reset_button.pack()

        feedbackFrame = tk.LabelFrame(self._root, text="Feedback")
        feedbackFrame.pack()

        self.at_start_checkbox = tk.Checkbutton(feedbackFrame, state=tk.DISABLED, text="At Start")
        self.at_start_checkbox.pack()
        self.at_end_checkbox = tk.Checkbutton(feedbackFrame, state=tk.DISABLED, text="At End")
        self.at_end_checkbox.pack()
        self.lockout_checkbox = tk.Checkbutton(feedbackFrame, state=tk.DISABLED, text="In Lockout")
        self.lockout_checkbox.pack()
        self.error_message = tk.Message(feedbackFrame, state=tk.DISABLED, text="Current Error", textvariable=self.var_error_msg)
        self.error_message.pack()
        self.token_message = tk.Message(feedbackFrame, state=tk.DISABLED, text="Current Tokens", textvariable=self.var_tokens_msg)
        self.token_message.pack()

        statusFrame = tk.LabelFrame(self._root, text="Status")
        statusFrame.pack()

        self.status_uuid = tk.Entry(statusFrame, state=tk.DISABLED, text="UUID", textvariable=self.var_status_uuid)
        self.status_uuid.pack()
        self.status_start_time = tk.Entry(statusFrame, state=tk.DISABLED, text="Start Time", textvariable=self.var_status_start_time)
        self.status_start_time.pack()
        self.status_previous_time = tk.Entry(statusFrame, state=tk.DISABLED, text="Previous Time", textvariable=self.var_status_prev_time)
        self.status_previous_time.pack()
        self.status_current_time = tk.Entry(statusFrame, state=tk.DISABLED, text="Current Time", textvariable=self.var_status_curr_time)
        self.status_current_time.pack()
        self.status_stop_time = tk.Entry(statusFrame, state=tk.DISABLED, text="Stop Time", textvariable=self.var_status_stop_time)
        self.status_stop_time.pack()

        self._controls = RobotControlInterface(
            at_start_cb=self._at_start_cb, 
            at_end_cb=self._at_end_cb, 
            lockout_cb=self._lockout_cb, 
            tokens_cb=self._tokens_cb, 
            status_cb=self._status_cb, 
            error_cb=self._error_cb)

    def spin(self):
        rospy.on_shutdown(self._shutdown_cb)
        self._root.mainloop()

    def _shutdown_cb(self):
        self._root.destroy()

    #===========================================================================
    #   User Controls
    #===========================================================================

    def _on_play_cb(self):
        self._controls.play()

    def _on_stop_cb(self):
        self._controls.stop()

    def _on_pause_cb(self):
        self._controls.pause()

    def _on_reset_cb(self):
        self._controls.reset()

    #===========================================================================
    #   Robot Control Feedback
    #===========================================================================

    def _at_start_cb(self, state):
        if state:
            self.at_start_checkbox.select()
        else:
            self.at_start_checkbox.deselect()

    def _at_end_cb(self, state):
        if state:
            self.at_end_checkbox.select()
        else:
            self.at_end_checkbox.deselect()

    def _lockout_cb(self, state):
        if state:
            self.lockout_checkbox.select()
        else:
            self.lockout_checkbox.deselect()

    def _status_cb(self, msg):
        self.var_status_uuid.set(msg.uuid)
        self.var_status_start_time.set(str(msg.start_time))
        self.var_status_prev_time.set(str(msg.previous_time))
        self.var_status_curr_time.set(str(msg.current_time))
        self.var_status_stop_time.set(str(msg.stop_time))

    def _tokens_cb(self, tokens_dct):
        self.var_tokens_msg.set(json.dumps(tokens_dct, sort_keys=True, indent=4))

    def _error_cb(self, str_msg):
       self.var_error_msg.set(str_msg)


if __name__ == "__main__":
    rospy.init_node('test_robot_control_server')

    node = TestRobotControlServerNode()
    node.spin()
