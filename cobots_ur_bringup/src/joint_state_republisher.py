#!/usr/bin/env python

import rospy
import roslibpy

from sensor_msgs.msg import JointState


class JointStateRepublisher:

    def __init__(self, rosbridge_host, rosbridge_port, bridge_name_prefix):
        self._bridge_client = roslibpy.Ros(host=rosbridge_host,port=rosbridge_port)

        self._joint_state_sub = rospy.Subscriber('{}/joint_states'.format(bridge_name_prefix),JointState,self._joint_state_bridge_cb)
        self._joint_state_pub = roslibpy.Topic(self._bridge_client, '{}/joint_states'.format(bridge_name_prefix), 'sensor_msgs/JointState')

    def _joint_state_bridge_cb(self, msg):
        self._joint_state_pub.publish({
            'name': msg.name,
            'position': msg.position,
            'velocity': msg.velocity,
            'effort': msg.effort
        })


if __name__ == "__main__":
    rospy.init_node('joint_state_republisher')

    rosbridge_host = rospy.get_param('~rosbridge_host',None)
    rosbridge_port = rospy.get_param('~rosbridge_port',None)
    bridge_name_prefix = rospy.get_param('~bridge_name_prefix',None)

    node = JointStateRepublisher(rosbridge_host, rosbridge_port, bridge_name_prefix)

    rospy.spin()
