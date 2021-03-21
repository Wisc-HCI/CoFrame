#!/usr/bin/env python

'''
Generates transforms to connect external application into ros frame

Frames:
- application_camera
- application (ros frame)
- control_target
'''

#TODO update for new UI system

import tf
import rospy

from geometry_msgs.msg import PoseStamped


DEFAULT_ROS_FRAME_ID = 'world'


class ApplicationTfHandlerNode:

    def __init__(self, ros_frame_id):
        self._ros_frame_id = ros_frame_id
        self._tf_br = tf.TransformBroadcaster()
        self._ros_frame_sub = rospy.Subscriber('application/ros_frame',PoseStamped,self.ros_frame_cb)
        self._camera_pose_sub = rospy.Subscriber('application/camera_pose',PoseStamped,self.camera_pose_frame_cb)
        self._control_target_pose_sub = rospy.Subscriber('application/control_target_pose',PoseStamped,self.control_target_pose_cb)

    def ros_frame_cb(self, msg):
        pos = msg.pose.position
        ort = msg.pose.orientation
        self._tf_br.sendTransform([pos.x,pos.y,pos.z],
                                  [ort.x,ort.y,ort.z,ort.w],
                                  rospy.Time.now(),
                                  self._ros_frame_id,
                                  msg.header.frame_id)

    def camera_pose_frame_cb(self, msg):
        pos = msg.pose.position
        ort = msg.pose.orientation
        self._tf_br.sendTransform([pos.x,pos.y,pos.z],
                                  [ort.x,ort.y,ort.z,ort.w],
                                  rospy.Time.now(),
                                  'application_camera',
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

    ros_frame_id = rospy.get_param('~ros_frame_id',DEFAULT_ROS_FRAME_ID)

    node = ApplicationTfHandlerNode(ros_frame_id)

    rospy.spin()
