#!/usr/bin/env python

import time
import rospy
import actionlib

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from cobots_core.msg import Stop, Servo, Move, Grip
from geometry_msgs.msg import Pose, Quaternion, Vector3
from cobots_core.msg import MoveTrajectoryAction, MoveTrajectoryGoal


DEFAULT_TEST_START_DELAY = 5


class SubSystemTest:

    def __init__(self, system_prefix, test_start_delay):
        self._system_prefix = system_prefix
        self._test_start_delay = test_start_delay
        self._received_msg_count = 0
        self._last_js_msg = None
        self._last_js_msg_time = None

        self._freedrive_pub = rospy.Publisher('{}/robot_control/freedrive'.format(system_prefix),Bool,queue_size=5)
        self._servoing_pub = rospy.Publisher('{}/robot_control/servoing'.format(system_prefix),Servo,queue_size=5)
        self._stop_pub = rospy.Publisher('{}/robot_control/stop'.format(system_prefix),Stop,queue_size=5)

        self._joint_state_sub = rospy.Subscriber('{}/joint_states'.format(system_prefix),JointState,self._joint_state_cb)

        self._move_trajectory_ac = actionlib.SimpleActionClient('{}/robot_control/move_trajectory'.format(system_prefix),MoveTrajectoryAction)
        self._gripper_pub = rospy.Publisher('{}/robot_control/gripper'.format(system_prefix),Grip,queue_size=5)

    def _joint_state_cb(self, msg):
        self._last_js_msg = msg
        self._last_js_msg_time = time.time()
        self._received_msg_count += 1

    def run(self):
        print 'Prefix:', self._system_prefix

        # Define Start Move Trajectory
        start_move = Move()
        start_move.motion_type = Move.LINEAR
        start_move.target_pose = Pose(
            position=Vector3(
                x=0.25,
                y=0.25,
                z=0.25),
            orientation=Quaternion(
                x=0,
                y=0,
                z=0,
                w=1))
        start_move.radius = Move.STD_RADIUS
        start_move.acceleration = Move.STD_ACCELERATION / 2
        start_move.velocity = Move.STD_VELOCITY / 2
        start_move.time = Move.STD_TIME

        start_trajectory_goal = MoveTrajectoryGoal()
        start_trajectory_goal.moves = [start_move]
        start_trajectory_goal.wait_for_finish = True

        # Define Stop Move Trajectory
        stop_move = Move()
        stop_move.motion_type = Move.LINEAR
        stop_move.target_pose = Pose(
            position=Vector3(
                x=0.25,
                y=-0.25,
                z=0.25),
            orientation=Quaternion(
                x=0,
                y=0,
                z=0,
                w=1))
        stop_move.radius = Move.STD_RADIUS
        stop_move.acceleration = Move.STD_ACCELERATION / 2
        stop_move.velocity = Move.STD_VELOCITY / 2
        stop_move.time = Move.STD_TIME

        stop_trajectory_goal = MoveTrajectoryGoal()
        stop_trajectory_goal.moves = [stop_move]

        # Define Servoing Message
        servo = Servo()
        servo.motion_type = Servo.CIRCULAR
        servo.target_pose = Pose(
            position=Vector3(
                x=-0.25,
                y=0.25,
                z=0.25),
            orientation=Quaternion(
                x=0,
                y=0,
                z=0,
                w=1))
        servo.radius = Servo.STD_RADIUS
        servo.acceleration = Servo.STD_ACCELERATION
        servo.velocity = Servo.STD_VELOCITY

        # Define Stop Message
        stop = Stop()
        stop.motion_type = Stop.LINEAR
        stop.acceleration = Stop.STD_ACCELERATION

        # Define Gripper Close Command
        gripper_open_goal = Grip()
        gripper_open_goal.position = 0.0
        gripper_open_goal.effort = 1
        gripper_open_goal.speed = 1

        # Define Gripper Open Command
        gripper_close_goal = Grip()
        gripper_close_goal.position = 0.85
        gripper_close_goal.effort = 1
        gripper_close_goal.speed = 1

        # Waiting for UR Controller
        print 'Waiting for Move Trajectory Server for UR Controller to connect'
        self._move_trajectory_ac.wait_for_server()

        # Interact with UR Controller
        print 'Sending move trajectory to start location'
        self._move_trajectory_ac.send_goal(start_trajectory_goal)
        self._move_trajectory_ac.wait_for_result()
        result = self._move_trajectory_ac.get_result()
        print 'Status: {}, Message: {}'.format(result.status,result.message)

        print 'Sending freedrive active, (waiting 30 seconds)'
        self._freedrive_pub.publish(Bool(True))
        rospy.sleep(30)

        print 'Sending freedrive disabled, (waiting 30 seconds)'
        self._freedrive_pub.publish(Bool(False))
        rospy.sleep(30)

        print 'Sending servoing command'
        self._servoing_pub.publish(servo)
        rospy.sleep(5)

        print 'Sending move trajectory to stop location'
        self._move_trajectory_ac.send_goal(stop_trajectory_goal)
        self._move_trajectory_ac.wait_for_result()
        result = self._move_trajectory_ac.get_result()
        print 'Status: {}, Message: {}'.format(result.status,result.message)
        rospy.sleep(1.25)

        print 'Sending stop command'
        self._stop_pub.publish(stop)
        rospy.sleep(1)

        # Interact with Gripper
        print 'Sending a close gripper command'
        self._gripper_pub.publish(gripper_close_goal)
        rospy.sleep(5)

        print 'Sending an open gripper command'
        self._gripper_pub.publish(gripper_open_goal)
        rospy.sleep(5)

        # Verifying joint position is being published
        print 'Checking that joint state messages are being published'
        print 'Number of messages received:', self._received_msg_count
        print 'Last message receive', '{} seconds ago'.format(time.time() - self._last_js_msg_time) if self._last_js_msg_time != None else 'never'
        print 'Contents of last message', self._last_js_msg


if __name__ == "__main__":
    rospy.init_node('bringup_test')

    test_start_delay = rospy.get_param('~test_start_delay',DEFAULT_TEST_START_DELAY)

    # Delay according to setting
    print 'Delaying test for {} seconds'.format(test_start_delay)
    rospy.sleep(test_start_delay)

    print '======================================================================'

    physicalTest = SubSystemTest('physical', 0)
    physicalTest.run()

    print '======================================================================'

    simulatedTest = SubSystemTest('simulated', 0)
    simulatedTest.run()

    print '======================================================================'

    plannerTest = SubSystemTest('planner', 0)
    plannerTest.run()

    print '======================================================================'
