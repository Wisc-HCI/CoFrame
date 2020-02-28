#!/usr/bin/env python

from interfaces.data_client_interface import DataClientInterface


DEFAULT_ROS_FRAME_ID = 'world'
DEFAULT_PUBLISH_RATE = 5.0


class DataModelRvizPublisherNode:

    def __init__(self, ros_frame_id, publish_rate):
        self._ros_frame_id = ros_frame_id
        self._tf_br = tf.TransformBroadcaster()
        self._data_client = DataClientInterface()
        self._update_timer = rospy.Timer(rospy.Duration(1.0/publish_rate), self._update_tf_cb)

    def _update_tf_cb(self, event):
        # get all locations

        # get all


if __name__ == "__main__":
    rospy.init_node('data_model_rviz_publisher')

    ros_frame_id = rospy.get_param('~ros_frame_id',DEFAULT_ROS_FRAME_ID)
    publish_rate = rospy.get_param('~publish_rate',DEFAULT_PUBLISH_RATE)

    node = DataModelRvizPublisherNode(ros_frame_id)

    rospy.spin()
