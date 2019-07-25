#!/usr/bin/env python

import rospy


class FakeCNC:

    def __init__(self):
        pass


if __name__ == "__main__":
    rospy.init_node('fake_cnc')
    node = FakeCNC()
    rospy.spin()
