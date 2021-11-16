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
        self._root = tk.Tk()
        self._root.resizable(True, True)

        self.var_error_msg = tk.StringVar()
        self.var_tokens_msg = tk.StringVar()
        self.var_status_uuid = tk.StringVar()
        self.var_status_start_time = tk.StringVar()
        self.var_status_prev_time = tk.StringVar()
        self.var_status_curr_time = tk.StringVar()
        self.var_status_stop_time = tk.StringVar()

        mainFrame = tk.Frame(self._root)
        mainFrame.grid(row=0)

        controlFrame = tk.LabelFrame(mainFrame, text="Controls")
        controlFrame.pack(side=tk.LEFT, fill=tk.Y)

        self.play_button = tk.Button(controlFrame, text="Play", command=self._on_play_cb)
        self.play_button.pack(fill=tk.X)
        self.stop_button = tk.Button(controlFrame, text="Stop", command=self._on_stop_cb)
        self.stop_button.pack(fill=tk.X)
        self.pause_button = tk.Button(controlFrame, text="Pause", command=self._on_pause_cb)
        self.pause_button.pack(fill=tk.X)
        self.reset_button = tk.Button(controlFrame, text="Reset", command=self._on_reset_cb)
        self.reset_button.pack(fill=tk.X)

        feedbackFrame = tk.LabelFrame(mainFrame, text="Feedback")
        feedbackFrame.pack(side=tk.LEFT, fill=tk.Y)

        self.at_start_checkbox = tk.Checkbutton(feedbackFrame, state=tk.DISABLED, text="At Start")
        self.at_start_checkbox.pack(fill=tk.X)
        self.at_end_checkbox = tk.Checkbutton(feedbackFrame, state=tk.DISABLED, text="At End")
        self.at_end_checkbox.pack(fill=tk.X)
        self.lockout_checkbox = tk.Checkbutton(feedbackFrame, state=tk.DISABLED, text="In Lockout")
        self.lockout_checkbox.pack(fill=tk.X)

        statusFrame = tk.LabelFrame(mainFrame, text="Status")
        statusFrame.pack(side=tk.LEFT, fill=tk.Y)

        status_uuid_lbl = tk.Label(statusFrame, text="UUID")
        status_uuid_lbl.grid(row=0, column=0)
        self.status_uuid = tk.Entry(statusFrame, state=tk.DISABLED, textvariable=self.var_status_uuid)
        self.status_uuid.grid(row=0, column=1)

        status_start_time_lbl = tk.Label(statusFrame, text="Start Time")
        status_start_time_lbl.grid(row=1, column=0)
        self.status_start_time = tk.Entry(statusFrame, state=tk.DISABLED, textvariable=self.var_status_start_time)
        self.status_start_time.grid(row=1, column=1)

        status_previous_time_lbl = tk.Label(statusFrame, text="Previous Time")
        status_previous_time_lbl.grid(row=2, column=0)
        self.status_previous_time = tk.Entry(statusFrame, state=tk.DISABLED, textvariable=self.var_status_prev_time)
        self.status_previous_time.grid(row=2, column=1)

        status_current_time_lbl = tk.Label(statusFrame, text="Current Time")
        status_current_time_lbl.grid(row=3, column=0)
        self.status_current_time = tk.Entry(statusFrame, state=tk.DISABLED, textvariable=self.var_status_curr_time)
        self.status_current_time.grid(row=3, column=1)

        status_stop_time_lbl = tk.Label(statusFrame, text="Stop Time")
        status_stop_time_lbl.grid(row=4, column=0)
        self.status_stop_time = tk.Entry(statusFrame, state=tk.DISABLED, textvariable=self.var_status_stop_time)
        self.status_stop_time.grid(row=4, column=1)

        messageFrame = tk.LabelFrame(self._root, text="Message")
        messageFrame.grid(row=1, sticky='EWNS')

        error_message_lbl = tk.Label(messageFrame, text="Current Error")
        error_message_lbl.pack()
        self.error_message = tk.Entry(messageFrame, state=tk.DISABLED, textvariable=self.var_error_msg)
        self.error_message.pack(expand=True, fill=tk.X)

        token_message_lbl = tk.Label(messageFrame, text="Current Tokens")
        token_message_lbl.pack()
        self.token_message = tk.Entry(messageFrame, state=tk.DISABLED, textvariable=self.var_tokens_msg)
        self.token_message.pack(expand=True, fill=tk.X)
        
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
