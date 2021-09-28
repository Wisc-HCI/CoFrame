#!/usr/bin/env python3

import tf
import json
import rospy



if __name__ == "__main__":
    rospy.init_node('replay_gripper_tfs')

    filename = rospy.get_params('~file')

    rospy.sleep(1)

    broadcaster = tf.TransformBroadcaster()

    with open(filename,'r') as f:
        data = json.load(f)

        index = 0
        while not rospy.is_shutdown():

            for toFrame, vals in data['capture'].items():
                pos = vals[index][0]
                rot = vals[index][1]

                for (fromFrame, _tl) in data['subtree']:
                    if _tl == toFrame:
                        br.sendTransform(pos, rot,
                                        rospy.Time.now(),
                                        toFrame, fromFrame)
            
            rospy.sleep(1)
            index += 1
            if index >= len(data['cmd']):
                index = 0