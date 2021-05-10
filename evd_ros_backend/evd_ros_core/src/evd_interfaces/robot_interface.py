'''
Wraps the robot bringup subsystem for each robot instance. This is expected to
interact with a UR robot using URScript.
'''


import time
import rospy
import actionlib

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from ur_msgs.msg import RobotModeDataMsg
from robotiq_85_msgs.msg import GripperStat
from evd_ros_core.msg import MoveTrajectoryAction
from evd_ros_core.msg import Stop, Servo, Move, Grip

from evd_script.data.trajectory import *


class RobotInterface:

    def __init__(self, prefix):
        self._prefix = prefix
        self._last_js_msg = JointState()
        self._last_js_msg_time = 0
        self._last_rms_msg = RobotModeDataMsg()
        self._last_rms_msg_time = 0
        self._last_gstat_msg = GripperStat()
        self._last_gstat_msg_time = 0

        self.freedrive_pub = rospy.Publisher('{}/robot_control/freedrive'.format(self._prefix),Bool,queue_size=5)
        self.servoing_pub = rospy.Publisher('{}/robot_control/servoing'.format(self._prefix),Servo,queue_size=5)
        self.stop_pub = rospy.Publisher('{}/robot_control/stop'.format(self._prefix),Stop,queue_size=5)
        self.grip_pub = rospy.Publisher('{}/robot_control/gripper'.format(self._prefix),Grip,queue_size=5)

        self._joint_state_sub = rospy.Subscriber('{}/joint_states'.format(self._prefix),JointState,self._joint_state_cb)
        self._robot_mode_state_sub = rospy.Subscriber('{}/ur_driver/robot_mode_state'.format(self._prefix),RobotModeDataMsg,self._robot_mode_state_cb)
        self._gripper_state_sub = rospy.Subscriber('{}/gripper/stat'.format(self._prefix), GripperStat, self._gripper_state_cb)

        self.move_trajectory_action = actionlib.SimpleActionClient('{}/robot_control/move_trajectory'.format(self._prefix),MoveTrajectoryAction)

    def _joint_state_cb(self, msg):
        self._last_js_msg = msg
        self._last_js_msg_time = time.time()

    def get_last_joint_state(self):
        return self._last_js_msg, self._last_js_msg_time

    def _robot_mode_state_cb(self, msg):
        self._last_rms_msg = msg
        self._last_rms_msg_time = time.time()

    def get_last_robot_mode_state(self):
        return self._last_rms_msg, self._last_rms_msg_time

    def _gripper_state_cb(self, msg):
        self._last_gstat_msg = msg
        self._last_gstat_msg_time = time.time()

    def get_last_gripper_state(self):
        return self._last_gstat_msg, self._last_gstat_msg_time

    def estop(self):
        msg = RobotInterface.pack_robot_stop_joint()
        self.stop_pub.publish(msg)

    def pause(self, state):
        pass

    @classmethod
    def pack_robot_move_joint(cls, joints, radius=Move.STD_RADIUS, acceleration=Move.JOINT_ACCELERATION, velocity=Move.JOINT_VELOCITY):
        move = Move()
        move.motion_type = Move.JOINT
        move.use_ur_ik = False
        move.target_joints = joints
        move.radius = radius
        move.acceleration = acceleration
        move.velocity = velocity
        return move

    @classmethod
    def pack_robot_move_pose_joint(cls, pose, radius=Move.STD_RADIUS, acceleration=Move.STD_ACCELERATION, velocity=Move.STD_VELOCITY):
        move = Move()
        move.motion_type = Move.JOINT
        move.use_ur_ik = True
        move.target_pose = pose
        move.radius = radius
        move.acceleration = acceleration
        move.velocity = velocity
        return move

    @classmethod
    def pack_robot_move_pose_linear(cls, pose, radius=Move.STD_RADIUS, acceleration=Move.STD_ACCELERATION, velocity=Move.STD_VELOCITY):
        move = Move()
        move.motion_type = Move.LINEAR
        move.target_pose = pose
        move.radius = radius
        move.acceleration = acceleration
        move.velocity = velocity
        return move

    @classmethod
    def pack_robot_move_pose_circular(cls, pose, path_pose, radius=Move.STD_RADIUS, acceleration=Move.STD_ACCELERATION, velocity=Move.STD_VELOCITY):
        move = Move()
        move.motion_type = Move.CIRCULAR
        move.target_pose = pose
        move.path_pose = path_pose
        move.radius = radius
        move.acceleration = acceleration
        move.velocity = velocity
        return move

    @classmethod
    def pack_robot_move_pose_process(cls, pose, radius=Move.STD_RADIUS, acceleration=Move.STD_ACCELERATION, velocity=Move.STD_VELOCITY):
        move = Move()
        move.motion_type = Move.PROCESS
        move.target_pose = pose
        move.radius = radius
        move.acceleration = acceleration
        move.velocity = velocity
        return move

    @classmethod
    def pack_robot_servo_joint(cls, joints, acceleration=Servo.STD_ACCELERATION, velocity=Servo.STD_VELOCITY):
        servo = Servo()
        servo.motion_type = Servo.JOINT
        servo.use_ur_ik = False
        servo.target_joints = joints
        servo.acceleration = acceleration
        servo.velocity = velocity
        return servo

    @classmethod
    def pack_robot_servo_pose_joint(cls, pose, acceleration=Servo.STD_ACCELERATION, velocity=Servo.STD_VELOCITY):
        servo = Servo()
        servo.motion_type = Servo.JOINT
        servo.use_ur_ik = True
        servo.target_pose = pose
        servo.acceleration = acceleration
        servo.velocity = velocity
        return servo

    @classmethod
    def pack_robot_servo_pose_circular(cls, pose, radius=Servo.STD_RADIUS, acceleration=Servo.STD_ACCELERATION, velocity=Servo.STD_VELOCITY):
        servo = Servo()
        servo.motion_type = Servo.JOINT
        servo.use_ur_ik = True
        servo.target_pose = pose
        servo.radius = radius
        servo.acceleration = acceleration
        servo.velocity = velocity
        return servo

    @classmethod
    def pack_robot_stop_joint(cls, acceleration=Stop.STD_ACCELERATION):
        stop = Stop()
        stop.motion_type = Stop.JOINT
        stop.acceleration = acceleration
        return stop

    @classmethod
    def pack_robot_stop_linear(cls, acceleration=Stop.STD_ACCELERATION):
        stop = Stop()
        stop.motion_type = Stop.LINEAR
        stop.acceleration = acceleration
        return stop
