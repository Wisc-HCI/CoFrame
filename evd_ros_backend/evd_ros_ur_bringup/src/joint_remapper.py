#!/usr/bin/env python3

'''
Joint Remapper

Provides joint renaming behavior for custom URDFs.

Primarily used for non-standard prefix or postfix applied to joints in a robot.
Such that the original hardware driver no longer aligns with the joint names of
the robot. Typically these drivers assume a specific naming convention so when
it is violated, there is no good way to rename the joints within their software.

Hence this node receives input states from such nodes and converts them into
the correct naming form.

Note that this node can act only act in a many-to-one publishing relationship.
Spawn multiple for a one-to-one relationship.

Note can only transform to pattern:
    <prefix><original-name><postfix>
'''

import rospy

from sensor_msgs.msg import JointState


DEFAULT_PREFIX = ''
DEFAULT_POSTFIX = ''
DEFAULT_OUTPUT_TOPIC = 'joint_remapper/output'


class JointRemapper:

    def __init__(self, input_topics, output_topic, prefix, postfix):
        self._prefix = prefix
        self._postfix = postfix

        self._js_pub = rospy.Publisher(output_topic,JointState,queue_size=1)

        self._js_subs = []
        for topic in input_topics:
            self._js_subs.append(rospy.Subscriber(topic,JointState,self._js_cb,queue_size=1))

    def _js_cb(self, msg):
        for i in range(0,len(msg.name)):
            msg.name[i] = "{0}{1}{2}".format(self._prefix,msg.name[i],self._postfix)
        self._js_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node('joint_remapper')

    input_topics = rospy.get_param('~input_topics')
    output_topic = rospy.get_param('~output_topic',DEFAULT_OUTPUT_TOPIC)
    prefix = rospy.get_param('~prefix',DEFAULT_PREFIX)
    postfix = rospy.get_param('~postfix',DEFAULT_POSTFIX)
    node = JointRemapper(input_topics,output_topic,prefix,postfix)

    while not rospy.is_shutdown():
        rospy.spin()
