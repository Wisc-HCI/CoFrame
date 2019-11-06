#!/usr/bin/env python

'''
The data server creates a link between the application and the data backend of
the ROS interface.
'''

import json
import rospy

from std_msgs.msg import String

from cobots_datatypes.location import Location
from cobots_datatypes.geometry import Pose, Position, Orientation
from cobots_datatypes.primitive import Primitive
from cobots_datatypes.trajectory import Trajectory
from cobots_datatypes.waypoint import Waypoint


class DataServer:

    def __init__(self, task_filepath):
        self._task = {}


if __name__ == '__main__':
    rospy.init_node('data_server')

    node = DataServer()

    rospy.spin()
