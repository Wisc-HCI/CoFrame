#!/usr/bin/env python3

import tf
import json
import rospy

from visualization_msgs.msg import *
from interactive_markers.menu_handler import *
from interactive_markers.interactive_marker_server import *

from geometry_msgs.msg import Vector3, Quaternion, Pose


DEFAULT_BASE_FRAME = 'planner_base_link'
DEFAULT_EE_FRAME = 'planner_ee_link'


class PoseMarkerNode:

    def __init__(self, base_frame, ee_frame):
        self._base_frame = base_frame
        self._ee_frame = ee_frame

        # Marker
        self._marker_server = InteractiveMarkerServer("robot_controls")
        self._target_marker = self._make_target_marker()
        self._marker_server.insert(self._target_marker, self._marker_feedback)
        self._marker_server.applyChanges()

        self._listener = tf.TransformListener()
        self._target_pub = rospy.Publisher('lively_tk/target_pose',Pose,queue_size=10)

    def _marker_feedback(self, feedback):
        self._target_marker.pose = feedback.pose
        self._target_pub.publish(feedback.pose)

    def _make_marker(self):
        marker = Marker()
        marker.type = Marker.ARROW
        marker.pose.orientation = Quaternion(0,0,0,1)
        marker.scale.x = 0.125
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 0.5
        return marker

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
    rospy.init_node('pose_marker')

    base_frame = rospy.get_param('~base_frame',DEFAULT_BASE_FRAME)
    ee_frame = rospy.get_param('~ee_frame',DEFAULT_EE_FRAME)

    node = PoseMarkerNode(base_frame, ee_frame)
    rospy.spin()
