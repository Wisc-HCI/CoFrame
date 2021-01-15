#!/usr/bin/env python

import rospy
import tkinter as tk

from interfaces.data_client_interface import DataClientInterface


class ApplicationModelControl(tk.Frame):

    def __init__(self, master=None):
        super().__init__(master)
        self.master = master

        self._data_client = DataClientInterface(use_application_interface=True)

        self.optiong_button = tk.Button(self.master, text="Get Options", command=self._get_options_cb)
        self.options_listBox = tk.Listbox(self.master)
        self.save_button = tk.Button(self.master, text="", command=self._save_cb)
        self.filenameVar = tk.StringVar()
        self.filename_textbox = tk.Entry(self.master, width=15, textvariable=self.filenameVar)
        self.load_button = tk.Button(self.master, text="", command=self._load_cb)

        self.pack()
        self.optiong_button.pack()
        self.options_listBox.pack()
        self.save_button.pack()
        self.filename_textbox.pack()
        self.load_button.pack()

    def _get_options_cb(self):
        options, currentFile = self._data_client.get_application_options()

        self.options_listBox.delete(0,tk.END)
        for op in options:
            self.options_listBox.insert(op)

        self.filenameVar.set(currentFile)

    def _save_cb(self):
        status, msg = self._data_client.save_application(self.filenameVar.get())

        print 'Saved: {0} - {1}'.format(status,msg)

    def _load_cb(self):
        status, msg = self._data_client.load_application(self.options_listBox.get(tk.ACTIVE))

        print 'Loaded: {0} - {1}'.format(status,msg)


if __name__ == "__main__":
    root = tk.Tk()
    node = ApplicationModelControl(master=root)
    node.mainloop()
