#!/usr/bin/env python3

import tf
import json
import rospy

from evd_ros_core.evd_script import *

from visualization_msgs.msg import *
from interactive_markers.menu_handler import *
from interactive_markers.interactive_marker_server import *

from geometry_msgs.msg import Vector3, Quaternion, Pose


DEFAULT_BASE_FRAME = '/app'
DEFAULT_EE_FRAME = '/ee_link'


class LocationPlotter:

    def __init__(self, filepath, base_frame, ee_frame):
        self._base_frame = base_frame
        self._ee_frame = ee_frame
        self._filepath = filepath

        # Marker
        self._marker_server = InteractiveMarkerServer("robot_controls")
        self._target_marker = self._make_target_marker()
        self._marker_server.insert(self._target_marker, self._marker_feedback)
        self._marker_server.applyChanges()

        self._listener = tf.TransformListener()

    def _marker_feedback(self, feedback):
        print("Marker Feedback:", feedback, "\n\n")
        self._target_marker.pose = feedback.pose

    def spin(self):
        data = {
            'base_frame': self._base_frame,
            'ee_frame': self._ee_frame,
            'path': []
        }

        file = open(self._filepath,'w')

        try:
            while not rospy.is_shutdown():
                inStr = raw_input('Press enter to capture pose (or press q to quit)')
                if inStr.lower() == 'q':
                    break
                (pos, rot) = self._listener.lookupTransform(self._base_frame, self._ee_frame, rospy.Time(0))
                data['path'].append({'position': pos, 'orientation': rot})
        except:
            pass

        json.dump(data, file, indent=4)
        file.close()

    def _make_marker(self):
        loc = Location()
        return loc.to_ros_marker()

    def _make_target_marker(self):
        marker = InteractiveMarker()
        marker.header.frame_id = self._base_frame
        marker.pose.position = Vector3(0,0,0)
        marker.pose.orientation = Quaternion(0,0,0,1)
        marker.scale = 0.25
        marker.name = "pose target"
        marker.description = "Target EE pose for Arm"

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self._make_marker())
        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)

        return marker


if __name__ == "__main__":
    rospy.init_node('location_plotter')

    filepath = rospy.get_param('~filepath')
    base_frame = rospy.get_param('~base_frame',DEFAULT_BASE_FRAME)
    ee_frame = rospy.get_param('~ee_frame',DEFAULT_EE_FRAME)

    node = LocationPlotter(filepath, base_frame, ee_frame)
    node.spin()
