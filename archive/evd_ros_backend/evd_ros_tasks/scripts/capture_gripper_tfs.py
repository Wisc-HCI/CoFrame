#!/usr/bin/env python3

'''
Simple script to capture the gripper tf subtree for visualization
'''

import tf
import json
import rospy

from sensor_msgs.msg import JointState


def joint_mapping(percent):
    temp = (percent / 100.0) * 0.804
    if temp > 0.804:
        temp = 0.804
    elif temp < 0:
        temp = 0
    return temp


if __name__ == "__main__":
    rospy.init_node('capture_gripper_tfs')

    print('Waiting...')
    rospy.sleep(1) # wait for setup

    js_pub = rospy.Publisher('planner/joint_states_labeled',JointState,queue_size=5)
    listener = tf.TransformListener()

    subtree = [
        ('planner_base_link','planner_shoulder_link'),
        ('planner_shoulder_link','planner_upper_arm_link'),
        ('planner_upper_arm_link','planner_forearm_link'),
        ('planner_forearm_link','planner_wrist_1_link'),
        ('planner_wrist_1_link','planner_wrist_2_link'),
        ('planner_wrist_2_link','planner_wrist_3_link'),
        ('planner_wrist_3_link','planner_flange'),
        ('planner_flange','planner_tool0'),
        ('planner_tool0','planner_robotiq_85_base_link'),
        ('planner_robotiq_85_base_link','planner_robotiq_85_left_knuckle_link'),
        ('planner_robotiq_85_base_link','planner_robotiq_85_right_knuckle_link'),
        ('planner_robotiq_85_left_knuckle_link','planner_robotiq_85_left_finger_link'),
        ('planner_robotiq_85_right_knuckle_link','planner_robotiq_85_right_finger_link'),
        ('planner_robotiq_85_base_link','planner_robotiq_85_left_inner_knuckle_link'),
        ('planner_robotiq_85_base_link','planner_robotiq_85_right_inner_knuckle_link'),
        ('planner_robotiq_85_left_inner_knuckle_link','planner_robotiq_85_left_finger_tip_link'),
        ('planner_robotiq_85_right_inner_knuckle_link','planner_robotiq_85_right_finger_tip_link')
    ]

    cmd = []
    capture = {subtree[i][1]:[] for i in range(0,len(subtree))}

    print('Starting...')

    for i in range(0,21):
        j = joint_mapping(i/20 * 100)
        print('joint value', i, '->', j)

        js = JointState()
        js.name = ['planner_robotiq_85_left_knuckle_joint']
        js.position = [j]

        cmd.append(j)
        js_pub.publish(js)
        rospy.sleep(1) # wait for jogging (really its less than this but i don't care too much)

        for (_from, _to) in subtree:
            (trans, rot) = listener.lookupTransform(_from, _to, rospy.Time(0))
            capture[_to].append((trans,rot))

    pack = {
        'subtree': subtree,
        'cmd': cmd,
        'capture': capture
    }

    print('Saving...')
    with open('capture_gripper_log.json','w+') as f:
        json.dump(pack, f, indent=4, sort_keys=True)

    print('Done...')
    rospy.spin()