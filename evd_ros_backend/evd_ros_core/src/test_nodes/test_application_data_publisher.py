#!/usr/bin/env python

import rospy
import Tkinter as tk

from ..interfaces.data_client_interface import DataClientInterface


class TestApplicationDataPublisher:

    def __init__(self, _root=None):

        self._data_client = DataClientInterface(use_application_interface=True)
        self._options = None

        self._root = tk.Tk()
        frame = tk.Frame(self._root)
        frame.pack()

        self.optiong_button = tk.Button(frame, text="Get Options", command=self._get_options_cb)
        self.optiong_button.pack()

        self.options_listBox = tk.Listbox(frame)
        self.options_listBox.pack()

        self.save_button = tk.Button(frame, text="Save", command=self._save_cb)
        self.save_button.pack()

        self.filenameVar = tk.StringVar()
        self.filename_textbox = tk.Entry(frame, width=15, textvariable=self.filenameVar)
        self.filename_textbox.pack()

        self.load_button = tk.Button(frame, text="Load", command=self._load_cb)
        self.load_button.pack()

    def _get_options_cb(self):
        msg = self._data_client.get_application_options()

        if self.options_listBox.size() > 0:
            self.options_listBox.delete(0,tk.END)

        for op in msg.options:
            self.options_listBox.insert(tk.END, op.filename)

        self.filenameVar.set(msg.currently_loaded_filename)
        self._options = msg

    def _save_cb(self):
        status, msg = self._data_client.save_application(False,self.filenameVar.get())

        print 'Saved: {0} - {1}'.format(status,msg)

    def _load_cb(self):
        if self.options_listBox.get(tk.ACTIVE) == '':
            print 'Cannot load what is not selected'
        else:
            status, msg = self._data_client.load_application(self.options_listBox.get(tk.ACTIVE),'','',0)
            print 'Loaded: {0} - {1}'.format(status,msg)

    def spin(self):
        rospy.on_shutdown(self._shutdown_cb)

        rospy.sleep(5)

        self._get_options_cb()

        self._root.mainloop()

    def _shutdown_cb(self):
        self._root.destroy()


if __name__ == "__main__":

    node = TestApplicationDataPublisher()
    node.spin()
