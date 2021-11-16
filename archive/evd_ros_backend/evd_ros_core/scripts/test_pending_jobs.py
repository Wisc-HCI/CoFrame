#!/usr/bin/env python3

'''
GUI provides information on pending jobs from a processor
'''

import json
import rospy
import tkinter as tk


class TestPendingJobs:

    def __init__(self):
        self._root = tk.Tk()
        self._root.resizable(True, True)

        self.var_job_id = tk.StringVar

        mainFrame = tk.Frame(self._root)
        mainFrame.grid(row=0)



    def spin(self):
        rospy.on_shutdown(self._shutdown_cb)
        self._root.mainloop()

    def _shutdown_cb(self):
        self._root.destroy()


if __name__ == "__main__":
    rospy.init_node('test_pending_jobs')

    node = TestPendingJobs()
    node.spin()