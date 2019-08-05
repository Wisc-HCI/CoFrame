#!/usr/bin/env python

'''
Fake CNC Node
Author: Curt Henrichs
Date: 7-25-19

Fakes a CNC device for a machine tending task.

Socket Protocol for fake CNC machine
    - ?D
        - Returns 0 or 1 (boolean door state)
    - ?R
        - Returns 0 or 1 (boolean running state)
    - !D<value:[0,1]>
        - Returns 0 or 1 (boolean set status)
    - !R<value:[0,1]>
        - Returns 0 or 1 (boolean set status)pass

ROS Parameters
    - ~rate
        "Frequency in Hertz to check socket"

ROS Published Topics
    - running_led
    - waiting_led
    - warning_led
'''

# Expert Qs:
#   - IO control for door or IO for door?

import rospy
import socket

from std_msgs.msg import Bool


TCP_IP = ''
TCP_PORT = 5041
BUFFER_SIZE = 5
DEFAULT_NODE_RATE = 10


class FakeCNC:

    def __init__(self, rate):
        self._rate = rate
        self._running_status = False # F = idle, T = running
        self._door_status = False #F = open, T = closed

        self._running_led_pub = rospy.Publisher("running_led",Bool,queue_size=2)
        self._warning_led_pub = rospy.Publisher("warning_led",Bool,queue_size=2)
        self._waiting_led_pub = rospy.Publisher("waiting_led",Bool,queue_size=2)

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.bind((TCP_IP, TCP_PORT))
        self._socket.listen(0)

        rospy.sleep(0.01)
        self._running_led_pub.publish(Bool(False))
        self._warning_led_pub.publish(Bool(True))
        self._waiting_led_pub.publish(Bool(True))
        rospy.sleep(0.01)

        print 'setup complete'

    def spin(self):
        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():

            print 'waiting for connection'
            client, address = self._socket.accept()
            print 'connected'

            while not rospy.is_shutdown():
                try:
                    data = client.recv(BUFFER_SIZE)
                except:
                    break

                if not data:
                    break
                data = data.decode('utf-8')
                print data

                try:
                    self._process_raw(data)
                except:
                    print 'Failed to parse'

            client.close()
            rate.sleep()

    def _process_raw(self, data):
        if data[0] == '?':      # query
            self._process_get(data)
        elif data[0] == '!':    # asignment
            self._process_set(data)
        else:                   # invalid
            print 'Invalid Operation'

    def _process_get(self, data):
        if data[1] == 'D':      # Door
            client.send('({})'.format(int(self._door_status == True)))
        elif data[1] == 'R':    # Run
            client.send('({})'.format(int(self._running_status == True)))
        else:                   # invalid
            print 'Invalid Atrribute'

    def _process_set(self, data):
        if data[1] == 'D':      # Door
            value = True if data[2] == '1' else False

            if not self._door_status and value:     # Close door
                self._door_status = True
                self._warning_led_pub.publish(Bool(False))
                client.send('(1)')
            elif self._door_status and not value:   # Open door
                if self._running_status:
                    client.send('(0)') # must not be running
                else:
                    self._door_status = False
                    self._warning_led_pub.publish(Bool(True))
                    client.send('1')
            else:                                   # already in state
                client.send('(1)')
        elif data[1] == 'R':    # Run
            value = True if data[2] == '1' else False

            if not self._running_status and value:      # Activate
                if not self._door_status:
                    client.send('(0)') # door must be closed
                else:
                    self._running_status = True
                    self._running_led_pub.publish(Bool(True))
                    self._waiting_led_pub.publish(Bool(False))
                    client.send('(1)')
            elif self._running_status and not value:    # Deactivate
                self._running_status = False
                self._running_led_pub.publish(Bool(False))
                self._waiting_led_pub.publish(Bool(True))
                client.send('(1)')
            else:                                       # already in state
                client.send('(1)')
        else:                   # invalid
            print 'Invalid Atrribute'


if __name__ == "__main__":
    rospy.init_node('fake_cnc')
    rate = rospy.get_param('~rate',DEFAULT_NODE_RATE)
    node = FakeCNC(rate)
    node.spin()
