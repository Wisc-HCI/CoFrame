#!/usr/bin/env python

'''
Fake CNC Node
Author: Curt Henrichs
Date: 7-25-19

Fakes a CNC device for a machine tending task.

Socket Protocol
    - ?D
        - 0 or 1 (boolean door state)
    - ?R
        - 0 or 1 (boolean running state)
    - !D0 or !D1
        - 0 or 1 (boolean set status)
    - !R0 or !R1
        - 0 or 1 (boolean set status)
'''

import rospy
import socket

from std_msgs.msg import Bool


TCP_IP = '127.0.0.1'
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
        self._socket.bind(TCP_IP, TCP_PORT)
        self._socket.listen(1)

    def spin(self):
        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            (clientsocket, address) = serversocket.accept()
            data = clientsocket.recv(BUFFER_SIZE)
            if not data:
                break
            data = data.decode('utf-8')

            print data
            try:
                if data[0] == '?':      # query
                    if data[1] == 'D':      # Door
                        clientsocket.send('{}'.format(int(self._door_status == True)))
                    elif data[1] == 'R':    # Run
                        clientsocket.send('{}'.format(int(self._running_status == True)))
                    else:                   # invalid
                        print 'Invalid Atrribute'
                elif data[0] == '!':    # asignment
                    if data[1] == 'D':      # Door
                        value = True if data[2] == '1' else False

                        if not self._door_status and value:     # Close door
                            self._door_status = True
                            self._warning_led_pub.publish(Bool(False))
                            clientsocket.send('1')
                        elif self._door_status and not value:   # Open door
                            if self._running_status:
                                clientsocket.send('0') # must not be running
                            else:
                                self._door_status = False
                                self._warning_led_pub.publish(Bool(True))
                                clientsocket.send('1')
                        else:                                   # already in state
                            clientsocket.send('1')

                    elif data[1] == 'R':    # Run
                        value = True if data[2] == '1' else False

                        if not self._running_status and value:      # Activate
                            if not self._door_status:
                                clientsocket.send('0') # door must be closed
                            else:
                                self._running_status = True
                                self._running_led_pub.publish(Bool(True))
                                self._waiting_led_pub.publish(Bool(False))
                                clientsocket.send('1')
                        elif self._running_status and not value:    # Deactivate
                            self._running_status = False
                            self._running_led_pub.publish(Bool(False))
                            self._waiting_led_pub.publish(Bool(True))
                            clientsocket.send('1')
                        else:                                       # already in state
                            clientsocket.send('1')

                    else:                   # invalid
                        print 'Invalid Atrribute'
                else:                   # invalid
                    print 'Invalid Operation'
            except:
                print 'Failed to parse'

            clientsocket.close()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('fake_cnc')
    rate = rospy.param('~rate',DEFAULT_NODE_RATE)
    node = FakeCNC(rate)
    node.spin()
