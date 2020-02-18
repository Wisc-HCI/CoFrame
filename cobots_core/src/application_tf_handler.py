#!/usr/bin/env python

'''
Generates transforms to connect external application into ros frame
'''

#TODO update for new cobots UI system

import tf
import rospy

from geometry_msgs.msg import PoseStamped


DEFAULT_ROS_FRAME_ID = 'world'


def ros_frame_cb(msg, ros_frame_id):
    pos = msg.pose.position
    ort = msg.pose.orientation
    tf_br = tf.TransformBroadcaster()
    tf_br.sendTransform([pos.x,pos.y,pos.z],
                        [ort.x,ort.y,ort.z,ort.w],
                        rospy.Time.now(),
                        ros_frame_id,
                        msg.header.frame_id)

def camera_pose_frame_cb(msg):
    pos = msg.pose.position
    ort = msg.pose.orientation
    tf_br = tf.TransformBroadcaster()
    tf_br.sendTransform([pos.x,pos.y,pos.z],
                        [ort.x,ort.y,ort.z,ort.w],
                        rospy.Time.now(),
                        'application_camera',
                        msg.header.frame_id)


if __name__ == "__main__":
    rospy.init_node('application_tf_handler')

    ros_frame_id = rospy.get_param('~ros_frame_id',DEFAULT_ROS_FRAME_ID)
    ros_frame_sub = rospy.Subscriber('application/ros_frames',PoseStamped,ros_frame_cb,ros_frame_id)

    camera_pose_sub = rospy.Subscriber('application/camera_pose',PoseStamped,camera_pose_frame_cb)

    rospy.spin()
