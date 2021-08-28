#!/usr/bin/env python3

'''
Generates transforms to connect external application into ros frame

Frames:
- application_camera
- application (ros frame)
- control_target
'''

import tf
import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose


DEFAULT_POSE = Pose()
DEFAULT_POSE.orientation.w = 1


class ApplicationTfHandlerNode:

    def __init__(self):
        self._tf_br = tf.TransformBroadcaster()
        self._ros_frame_sub = rospy.Subscriber('application/app_to_ros_frame',PoseStamped,self.app_frame_cb)
        self._camera_pose_sub = rospy.Subscriber('application/camera_pose',PoseStamped,self.camera_pose_frame_cb)
        self._control_target_pose_sub = rospy.Subscriber('application/control_target_pose',PoseStamped,self.control_target_pose_cb)

        self.app_frame_cb(PoseStamped(header=Header(frame_id='world'), pose=DEFAULT_POSE))
        self.camera_pose_frame_cb(PoseStamped(header=Header(frame_id='app'), pose=DEFAULT_POSE))
        self.control_target_pose_cb(PoseStamped(header=Header(frame_id='app'), pose=DEFAULT_POSE))

    def app_frame_cb(self, msg):
        pos = msg.pose.position
        ort = msg.pose.orientation
        self._tf_br.sendTransform([pos.x,pos.y,pos.z],
                                  [ort.x,ort.y,ort.z,ort.w],
                                  rospy.Time.now(),
                                  'app',
                                  msg.header.frame_id)

    def camera_pose_frame_cb(self, msg):
        pos = msg.pose.position
        ort = msg.pose.orientation
        self._tf_br.sendTransform([pos.x,pos.y,pos.z],
                                  [ort.x,ort.y,ort.z,ort.w],
                                  rospy.Time.now(),
                                  'visualization_camera',
                                  msg.header.frame_id)

    def control_target_pose_cb(self, msg):
        pos = msg.pose.position
        ort = msg.pose.orientation
        self._tf_br.sendTransform([pos.x,pos.y,pos.z],
                                  [ort.x,ort.y,ort.z,ort.w],
                                  rospy.Time.now(),
                                  'control_target',
                                  msg.header.frame_id)


if __name__ == "__main__":
    rospy.init_node('application_tf_handler')
    node = ApplicationTfHandlerNode()
    rospy.spin()
