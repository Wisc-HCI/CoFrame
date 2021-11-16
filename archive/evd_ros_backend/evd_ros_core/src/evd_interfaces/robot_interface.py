'''
Robot interface defines an abstracted single-arm with single gripper robot for EvD.

Basic robot behavior to move to a joint state or a pose is assumed. Gripper with
position (and optional speed/effort) is assumed.

Base robot is conceptualized around a UR3e with Robotiq 85 2-Finger gripper.

The asynchronous methods use the ACK/NACK as feedback channel whereas the synchronous
methods use services. All robot behavior is also being presented in a generalized 
status update pushed from the robot implementation.

Class methods are provided to simplify packing messages. Though it is probably best not
to use them outside of this interface.
'''

import rospy

from evd_ros_core.srv import SetRobotMove, SetRobotMoveTrajectory, SetRobotGrip
from evd_ros_core.msg import RobotAck, RobotStatus, RobotStop, RobotPause, RobotMove, RobotMoveTrajectory, RobotGrip, RobotInitialize


class RobotInterface:

    def __init__(self, prefix=None):
        self._prefix = prefix
        prefix_fmt = prefix+'/' if prefix != None else ''
        self._latest_status = None
        self._ack_table = {}

        self.init_pub = rospy.Publisher('{0}robot/initialize'.format(prefix_fmt), RobotInitialize, queue_size=10)
        self.estop_pub = rospy.Publisher('{0}robot/estop'.format(prefix_fmt), RobotStop, queue_size=10)
        self.pause_pub = rospy.Publisher('{0}robot/pause'.format(prefix_fmt), RobotPause, queue_size=10)
        self.move_pub = rospy.Publisher('{0}robot/move'.format(prefix_fmt), RobotMove, queue_size=10)
        self.move_trajectory_pub = rospy.Publisher('{0}robot/move_trajectory'.format(prefix_fmt), RobotMoveTrajectory, queue_size=10)
        self.grip_pub = rospy.Publisher('{0}robot/grip'.format(prefix_fmt), RobotGrip, queue_size=10)

        self.move_srv = rospy.ServiceProxy('{0}robot/set_move'.format(prefix_fmt),SetRobotMove)
        self.move_trajectory_srv = rospy.ServiceProxy('{0}robot/set_move_trajectory'.format(prefix_fmt),SetRobotMoveTrajectory)
        self.grip_srv = rospy.ServiceProxy('{0}robot/set_grip'.format(prefix_fmt),SetRobotGrip)

        self.ack_sub = rospy.Subscriber('{0}robot/ack'.format(prefix_fmt), RobotAck, self._ack_cb)
        self.status_sub = rospy.Subscriber('{0}robot/wait'.format(prefix_fmt), RobotStatus, self._status_cb)

    def _ack_cb(self, msg):
        self._ack_table[msg.subsystem] = msg.ack

    def _status_cb(self, msg):
        self._latest_status = msg

    '''
    Public Methods
        - Exposes robot behavior
        - Some methods (e.g., pause may not be implemented for all robots)
    '''

    def initialize(self, gripper_position=None, arm_joints=None):
        msg = RobotInitialize()
        msg.gripper_position = gripper_position if gripper_position != None else -1
        msg.arm_joints = arm_joints if arm_joints != None else []
        self.init_pub.publish(msg)

    def estop(self):
        self.estop_pub.publish(RobotStop(True))

    def pause(self, state):
        self.pause_pub.publish(RobotPause(state))

    def move_async(self, evd_waypoint, move_type='ee_ik', velocity=None, manual_safety=False):
        msg = RobotInterface.convert_waypoint_to_move(move_type, evd_waypoint, velocity, manual_safety)
        self.move_pub.publish(msg)

    def move_sync(self, evd_waypoint, move_type='ee_ik', velocity=None, manual_safety=False):
        goal = RobotInterface.convert_waypoint_to_move(move_type, evd_waypoint, velocity, manual_safety)
        return self.move_srv(goal).status

    def move_trajectory_async(self, evd_trajectory, manual_safety=False):
        msg = RobotInterface.pack_move_trajectory(evd_trajectory,manual_safety)
        self.move_trajectory_pub.publish(msg)

    def move_trajectory_sync(self, evd_trajectory, manual_safety=False):
        goal = RobotInterface.pack_move_trajectory(evd_trajectory,manual_safety)
        return self.move_trajectory_srv(goal).status

    def grip_async(self, position, speed, effort, manual_safety=False):
        msg = RobotInterface.pack_gripper_msg(position,speed,effort,manual_safety)
        self.grip_pub.publish(msg)

    def grip_sync(self, position, speed, effort, manual_safety=False):
        goal = RobotInterface.pack_gripper_msg(position,speed,effort,manual_safety)
        return self.grip_srv(goal).status

    def is_acked(self, subsystem=None, clearEntry=True):
        subsystem = subsystem if subsystem != None else 'arm'

        if not subsystem in self._ack_table.keys():
            return None
        else:
            ack = self._ack_table[subsystem]
            if clearEntry:
                del self._ack_table[subsystem]
            return ack

    def get_status(self):
        return self._latest_status
    
    '''
    State Properties (from last status message)
    '''

    @property
    def current_pose(self):
        return self._latest_status.arm_pose

    @property
    def current_joints(self):
        return self._latest_status.arm_joints

    @property
    def current_gripper_position(self):
        return self._latest_status.gripper_position

    @property
    def is_real_robot(self):
        return self._latest_status.is_real_robot

    @property
    def is_in_emergency_stop(self):
        return self._latest_status.is_in_emergency_stop

    @property
    def is_running(self):
        return self._latest_status.is_running

    @property
    def gripper_object_detected(self):
        return self._latest_status.gripper_object_detected

    @property
    def status_flag(self):
        return self._latest_status.status

    '''
    Class Methods
        - Pack messages
    '''

    @classmethod
    def pack_move_trajectory(cls, evd_trajectory, manual_safety=False):
        moves = []

        mtype = evd_trajectory.move_type
        vel = evd_trajectory.velocity
       
        loc_uuid = evd_trajectory.start_location_uuid
        loc = evd_trajectory.context.get_location(loc_uuid)
        moves.append(cls.convert_waypoint_to_move(mtype, loc, vel, 0, manual_safety))

        for wp_uuid in evd_trajectory.waypoint_uuids:
            wp = evd_trajectory.context.get_waypoint(wp_uuid)
            moves.append(cls.convert_waypoint_to_move(mtype, wp, vel, 0, manual_safety))

        loc_uuid = evd_trajectory.end_location_uuid
        loc = evd_trajectory.context.get_location(loc_uuid)
        moves.append(cls.convert_waypoint_to_move(mtype, loc, vel, 0, manual_safety))
       
        msg = RobotMoveTrajectory()
        msg.moves = moves
        return msg

    @classmethod
    def convert_waypoint_to_move(cls, move_type, evd_waypoint, velocity=None, manual_safety=False):
        
        if move_type == 'joint':
            if evd_waypoint.joints != None and len(evd_waypoint.joints) > 0:
                move = cls.pack_move_joint_msg(evd_waypoint.joints, velocity, 0, manual_safety)
            else:
                raise Exception('Joints must be specified')

        elif move_type == 'ee_ik':
            move = cls.pack_move_ik_msg(evd_waypoint.to_ros(), velocity, 0, manual_safety)

        else:
            raise Exception('Invalid Move Type specified')

        return move

    @classmethod
    def pack_move_ik_msg(cls, pose, velocity=None, timestep=None, manual_safety=False):
        velocity = velocity if velocity != None else RobotMove.STD_VELOCITY
        timestep = timestep if timestep != None else RobotMove.STD_TIMESTEP

        msg = RobotMove()
        msg.motion_type = RobotMove.EE_IK
        msg.target_pose = pose
        msg.velocity = velocity
        msg.timestep = timestep
        msg.manual_safety = manual_safety
        return msg

    @classmethod
    def pack_move_joints_msg(cls, joints, velocity=None, timestep=None, manual_safety=False):
        velocity = velocity if velocity != None else RobotMove.JOINT_VELOCITY
        timestep = timestep if timestep != None else RobotMove.STD_TIMESTEP

        msg = RobotMove()
        msg.motion_type = RobotMove.JOINT
        msg.target_joints = joints
        msg.velocity = velocity
        msg.timestep = timestep
        msg.manual_safety = manual_safety
        return msg

    @classmethod
    def pack_gripper_msg(cls, position, speed, effort, manual_safety=False):
        if position < 0 or position > 100:
            raise Exception('Position out of range')
        elif speed <= 0:
            raise Exception('Speed must be a positive value (greater than zero)')
        elif effort < 0:
            raise Exception('Effort must be a positive value or zero')

        msg = RobotGrip()
        msg.position = position
        msg.speed = speed
        msg.effort = effort
        msg.manual_safety = manual_safety
        return msg