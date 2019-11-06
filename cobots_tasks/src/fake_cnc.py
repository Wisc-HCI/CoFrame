#!/usr/bin/env python

'''
Fake CNC Node
Author: Curt Henrichs
Date: 7-25-19






TODO redo this as a ROS string input/output for node








Fakes a CNC device for a machine tending task.

Socket Protocol for fake CNC machine
    - ?D
        - Returns 1 or 2 (boolean door state, 2 = True)
    - ?R
        - Returns 1 or 2 (boolean running state, 2 = True)
    - !D<value:[1,2]>
        - Returns 1 or 2 (boolean set status, 2 = True)
    - !R<value:[1,2]>
        - Returns 1 or 2 (boolean set status, 2 = True)

ROS Parameters
    - ~rate
        "Frequency in Hertz to check socket"

ROS Published Topics
    - running_led
    - waiting_led
    - warning_led
'''

import rospy
import socket
import traceback

from std_msgs.msg import Bool, UInt16


TCP_IP = ''
TCP_PORT = 5041
BUFFER_SIZE = 20
DEFAULT_NODE_RATE = 10
SETUP_DELAY = 5

DOOR_OPENED = 45
DOOR_CLOSED = 170

CNC_RUN_TIME = 5


class FakeCNC:

    def __init__(self, rate):
        self._rate = rate
        self._timer = None
        self._running_status = False # F = idle, T = running
        self._door_status = False #F = open, T = closed

        self._running_led_pub = rospy.Publisher("running_led",Bool,queue_size=1)
        self._warning_led_pub = rospy.Publisher("warning_led",Bool,queue_size=1)
        self._waiting_led_pub = rospy.Publisher("waiting_led",Bool,queue_size=1)
        self._door_servo_pub = rospy.Publisher("door_servo",UInt16, queue_size=1)

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.bind((TCP_IP, TCP_PORT))
        self._socket.listen(1)

    def _timer_cb(self, event):
        self._running_status = False
        self._running_led_pub.publish(Bool(False))
        self._waiting_led_pub.publish(Bool(True))

    def spin(self):

        rospy.sleep(SETUP_DELAY)
        self._running_led_pub.publish(Bool(False))
        self._warning_led_pub.publish(Bool(True))
        self._waiting_led_pub.publish(Bool(True))
        self._door_servo_pub.publish(UInt16(DOOR_OPENED))
        print 'setup complete'

        while not rospy.is_shutdown():

            print 'waiting for connection'
            client, address = self._socket.accept()
            print 'connected'

            while True:
                # receive raw data
                try:
                    data = client.recv(BUFFER_SIZE)
                    if not data:
                        break
                except:
                    break

                # convert to parsable
                data = data.decode('utf-8')
                print data

                # process
                try:
                    self._process_raw(client,data)
                except:
                    traceback.print_exc()
                    print 'Failed to parse'

                rospy.sleep(1/self._rate)

            client.close()
            rospy.sleep(1/self._rate)

    def _process_raw(self, client, data):
        if data[0] == '?':      # query
            self._process_get(client,data)
        elif data[0] == '!':    # asignment
            self._process_set(client,data)
        else:                   # invalid
            print 'Invalid Operation'
            client.send('(0)\n')

    def _process_get(self, client, data):
        if data[1] == 'D':      # Door
            client.send('({})\n'.format(2 if self._door_status else 1))
        elif data[1] == 'R':    # Run
            client.send('({})\n'.format(2 if self._running_status else 1))
        else:                   # invalid
            print 'Invalid Atrribute'
            client.send('(0)\n')

    def _process_set(self, client, data):
        if data[1] == 'D':      # Door
            value = True if data[2] == '2' else False

            if not self._door_status and value:     # Close door
                self._door_status = True
                self._warning_led_pub.publish(Bool(False))
                self._door_servo_pub.publish(UInt16(DOOR_CLOSED))
                client.send('(2)\n')
            elif self._door_status and not value:   # Open door
                if self._running_status:
                    client.send('(1)\n') # must not be running
                else:
                    self._door_status = False
                    self._warning_led_pub.publish(Bool(True))
                    self._door_servo_pub.publish(UInt16(DOOR_OPENED))
                    client.send('(2)\n')
            else:                                   # already in state
                client.send('(2)\n')
        elif data[1] == 'R':    # Run
            value = True if data[2] == '2' else False

            if not self._running_status and value:      # Activate
                if not self._door_status:
                    client.send('(1)\n') # door must be closed
                else:
                    self._timer = rospy.Timer(rospy.Duration(CNC_RUN_TIME), self._timer_cb, oneshot=True)

                    self._running_status = True
                    self._running_led_pub.publish(Bool(True))
                    self._waiting_led_pub.publish(Bool(False))
                    client.send('(2)\n')
            elif self._running_status and not value:    # Deactivate
                self._running_status = False
                self._running_led_pub.publish(Bool(False))
                self._waiting_led_pub.publish(Bool(True))
                client.send('(2)\n')
            else:                                       # already in state
                client.send('(2)\n')
        else:                   # invalid
            print 'Invalid Atrribute'
            client.send('(0)\n')


if __name__ == "__main__":
    rospy.init_node('fake_cnc')
    rate = rospy.get_param('~rate',DEFAULT_NODE_RATE)
    node = FakeCNC(rate)
    node.spin()
