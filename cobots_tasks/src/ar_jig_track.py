#!/usr/bin/env python

'''
AR Jig Track
Author: Curt Henrichs
Date: 7-25-19

Tracks AR tag attached to jig and report this on socket

Socket Protocol for retrieving Pose
    - P
        - Returns [px,py,pz,rx,ry,rz]

ROS Parameters
    - ~rate
        "Frequency in Hertz to check socket"
    - ~tag_id
        "ID number of tag being tracked"

ROS Subscribed Topics
    - /tf
'''

#Note right now map is not in the right spot for the robot, but this can be fixed

import tf
import rospy
import socket


TCP_IP = '127.0.0.1'
TCP_PORT = 5042
BUFFER_SIZE = 5
DEFAULT_NODE_RATE = 10


class ARJigTrack:

    def __init__(self, rate, tag_id):
        self._rate = rate
        self._tag_id = tag_id

        self._tf_listener = tf.TransformListener()

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
                if data[0] == 'P':
                    #lookup tf
                    try:
                        (pos, rot) = self._tf_listener.lookupTransform('ar_marker_{0}'.format(self._tag_id),'map', rospy.Time(0))
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        print 'failed to lookup tf'
                        clientsocket.send('')
                        break

                    # send info
                    msg = '[{0},{1},{2},'.format(pos[0],pos[1],pos[2])
                    msg += '{0},{1},{2}]'.format(rot[0],rot[1],rot[2])
                    clientsocket.send(msg)
                else:
                    print 'Invalid request'
            except:
                print 'Failed'

            clientsocket.close()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('ar_jig_track')
    rate = rospy.param('~rate',DEFAULT_NODE_RATE)
    tag_id = rospy.param('~tag_id')
    node = ARJigTrack(rate,tag_id)
    node.spin()
