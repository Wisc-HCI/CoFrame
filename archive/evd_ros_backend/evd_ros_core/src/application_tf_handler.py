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


class ApplicationTfHandlerNode:

    def __init__(self):
        self._app_pose = PoseStamped()
        self._app_pose.pose.orientation.w = 1
        self._app_pose.header.frame_id = 'world'
        self._camera_pose = PoseStamped()
        self._camera_pose.pose.orientation.w = 1
        self._camera_pose.header.frame_id = 'app'
        self._target_pose = PoseStamped()
        self._target_pose.pose.orientation.w = 1
        self._target_pose.header.frame_id = 'app'

        self._tf_br = tf.TransformBroadcaster()
        self._ros_frame_sub = rospy.Subscriber('application/app_to_ros_frame',PoseStamped,self.app_frame_cb)
        self._camera_pose_sub = rospy.Subscriber('application/camera_pose',PoseStamped,self.camera_pose_frame_cb)
        self._control_target_pose_sub = rospy.Subscriber('application/control_target_pose',PoseStamped,self.control_target_pose_cb)

    def app_frame_cb(self, msg):
        self._app_pose = msg

    def camera_pose_frame_cb(self, msg):
        self._camera_pose = msg

    def control_target_pose_cb(self, msg):
        self._target_pose = msg

    def spin(self):

        while not rospy.is_shutdown():

            pos = self._app_pose.pose.position
            ort = self._app_pose.pose.orientation
            self._tf_br.sendTransform([pos.x,pos.y,pos.z],
                                    [ort.x,ort.y,ort.z,ort.w],
                                    rospy.Time.now(),
                                    'app',
                                    self._app_pose.header.frame_id)

            pos = self._camera_pose.pose.position
            ort = self._camera_pose.pose.orientation
            self._tf_br.sendTransform([pos.x,pos.y,pos.z],
                                    [ort.x,ort.y,ort.z,ort.w],
                                    rospy.Time.now(),
                                    'visualization_camera',
                                    self._camera_pose.header.frame_id)

            pos = self._target_pose.pose.position
            ort = self._target_pose.pose.orientation
            self._tf_br.sendTransform([pos.x,pos.y,pos.z],
                                    [ort.x,ort.y,ort.z,ort.w],
                                    rospy.Time.now(),
                                    'control_target',
                                    self._target_pose.header.frame_id)

            rospy.sleep(0.25)


if __name__ == "__main__":
    rospy.init_node('application_tf_handler')
    node = ApplicationTfHandlerNode()
    node.spin()
