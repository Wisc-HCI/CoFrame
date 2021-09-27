#!/usr/bin/env python3

import json
import rospy

from std_msgs.msg import String

singleShot = False


def listener_fnt(msg):
    global singleShot

    if not singleShot:
        singleShot = True
        print(json.dumps(json.loads(msg.data), indent=4, separators=(',', ': ')))


if __name__ == "__main__":
    rospy.init_node('program_update_listener')

    rospy.Subscriber('program/update', String, listener_fnt, queue_size=1)

    rospy.spin()