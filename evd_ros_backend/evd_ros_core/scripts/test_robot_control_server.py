#!/usr/bin/env python3

'''
GUI provides controls for robot server.

- Controls to switch between robots
- *Freedrive not expected to be implemented
- Play controls
'''


import rospy
import tkinter as tk


from evd_interfaces.robot_control_interface import RobotControlInterface


class TestRobotControlServerNode:

    def __init__(self):
        self._root = tk.Tk()
        frame = tk.Frame(self._root)
        frame.pack()

        self.use_phy_checkbox = tk.Checkbutton(frame, text='Use Physical', command=self._on_use_phy_cb)
        self.use_phy_checkbox.pack()
        self.use_sim_checkbox = tk.Checkbutton(frame, text='Use Simulated', command=self._on_use_sim_cb)
        self.use_sim_checkbox.pack()
        self.freedrive_checkbox = tk.Checkbutton(frame, text='Freedrive', command=self._on_freedrive_cb)
        self.freedrive_checkbox.pack()

        self.play_button = tk.Button(frame, text="Play", command=self._on_play_cb)
        self.play_button.pack()
        self.stop_button = tk.Button(frame, text="Stop", command=self._on_stop_cb)
        self.stop_button.pack()
        self.pause_button = tk.Button(frame, text="Pause", command=self._on_pause_cb)
        self.pause_button.pack()
        self.reset_button = tk.Button(frame, text="Reset", command=self._on_reset_cb)
        self.reset_button.pack()

        self._controls = RobotControlInterface()

    def _on_use_phy_cb(self):
        self._controls.use_physical_robot(self.use_phy_checkbox.state())

    def _on_use_sim_cb(self):
        self._controls.use_simulated_robot(self.use_sim_checkbox.state())

    def _on_freedrive_cb(self):
        self._controls.freedrive(self.freedrive_checkbox.state())

    def _on_play_cb(self):
        self._controls.play()

    def _on_stop_cb(self):
        self._controls.stop()

    def _on_pause_cb(self):
        self._controls.pause()

    def _on_reset_cb(self):
        self._controls.reset()

    def spin(self):
        rospy.on_shutdown(self._shutdown_cb)
        self._root.mainloop()

    def _shutdown_cb(self):
        self._root.destroy()


if __name__ == "__main__":
    rospy.init_node('test_robot_control_server')

    node = TestRobotControlServerNode()
    node.spin()
