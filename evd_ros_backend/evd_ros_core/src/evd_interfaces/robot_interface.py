'''
TODO write this
'''

import rospy
import actionlib

from evd_ros_core.msg import RobotGripAction, RobotGripGoal
from evd_ros_core.msg import RobotMoveTrajectoryAction, RobotMoveTrajectoryGoal
from evd_ros_core.msg import RobotAck, RobotStatus, RobotStop, RobotPause, RobotMove, RobotGrip


class RobotInterface:

    def __init__(self, prefix=None):
        self._prefix = prefix
        prefix_fmt = prefix+'/' if prefix != None else ''
        self._latest_status = None
        self._ack_table = {}

        self.estop_pub = rospy.Publisher('{0}robot/estop'.format(prefix_fmt), RobotStop, queue_size=10)
        self.pause_pub = rospy.Publisher('{0}robot/pause'.format(prefix_fmt), RobotPause, queue_size=10)

        self.ack_sub = rospy.Subscriber('{0}robot/ack'.format(prefix_fmt), RobotAck, self._ack_cb)
        self.status_sub = rospy.Subscriber('{0}robot/wait'.format(prefix_fmt), RobotStatus, self._status_cb)

        self.move_action = actionlib.SimpleActionClient('{0}robot/action/move'.format(prefix_fmt), RobotMoveTrajectoryAction)
        self.grip_action = actionlib.SimpleActionClient('{0}robot/action/grip'.format(prefix_fmt), RobotGripAction)

    def _ack_cb(self, msg):
        self._ack_table[msg.subsystem] = msg.ack

    def _status_cb(self, msg):
        self._latest_status = msg

    def estop(self):
        self.estop_pub.publish(RobotStop(True))

    def pause(self, state):
        self.pause_pub.publish(RobotPause(True))

    def move_async(self, evd_waypoint, move_type='ee_ik', velocity=None, manual_safety=False):
        pass

    def move_sync(self, evd_waypoint, move_type='ee_ik', velocity=None, manual_safety=False):
        pass

    def move_trajectory_async(self, evd_trajectory, manual_safety=False):
        pass

    def move_trajectory_sync(self, evd_trajectory, manual_safety=False):
        pass

    def grip_async(self, position, speed, effort, manual_safety=False):
        pass

    def grip_sync(self, position, speed, effort, manual_safety=False):
        pass

    def servo_ik(self, pose):
        msg = RobotInterface.pack_servo_ik_msg(pose)
        self.servo_pub.publish(msg)

    def servo_joints(self, joints):
        msg = RobotInterface.pack_servo_joint_msg(joints)
        self.servo_pub.publish(msg)

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

    @classmethod
    def pack_trajectory_action_goal(cls, evd_trajectory, wait=True):
        moves = []

        mtype = evd_trajectory.move_type
        vel = evd_trajectory.velocity
       
        loc_uuid = evd_trajectory.start_location_uuid
        loc = evd_trajectory.context.get_location(loc_uuid)
        moves.append(cls.convert_waypoint_to_move(mtype, loc, vel))

        for wp_uuid in evd_trajectory.waypoint_uuids:
            wp = evd_trajectory.context.get_waypoint(wp_uuid)
            moves.append(cls.convert_waypoint_to_move(mtype, wp, vel))

        loc_uuid = evd_trajectory.end_location_uuid
        loc = evd_trajectory.context.get_location(loc_uuid)
        moves.append(cls.convert_waypoint_to_move(mtype, loc, vel))
       
        goal = RobotMoveTrajectoryGoal()
        goal.moves = moves
        goal.wait_for_finish = wait
        return goal

    @classmethod
    def convert_waypoint_to_move(cls, move_type, evd_waypoint, velocity=None):
        
        if move_type == 'joint':
            if evd_waypoint.joints != None and len(evd_waypoint.joints) > 0:
                move = cls.pack_move_joint_msg(evd_waypoint.joints,velocity)
            else:
                raise Exception('Joints must be specified')

        elif move_type == 'ee_ik':
            move = cls.pack_move_ik_msg(evd_waypoint.to_ros())

        else:
            raise Exception('Invalid Move Type specified')

        return move

    @classmethod
    def pack_move_ik_msg(cls, pose, velocity=None, timestep=None):
        velocity = velocity if velocity != None else RobotMove.STD_VELOCITY
        timestep = timestep if timestep != None else RobotMove.STD_TIMESTEP

        msg = RobotMove()
        msg.motion_type = RobotMove.EE_IK
        msg.target_pose = pose
        msg.velocity = velocity
        msg.timestep = timestep
        return msg

    @classmethod
    def pack_move_joints_msg(cls, joints, velocity=None, timestep=None):
        velocity = velocity if velocity != None else RobotMove.JOINT_VELOCITY
        timestep = timestep if timestep != None else RobotMove.STD_TIMESTEP

        msg = RobotMove()
        msg.motion_type = RobotMove.JOINT
        msg.target_joints = joints
        msg.velocity = velocity
        msg.timestep = timestep
        return msg

    @classmethod
    def pack_gripper_action_goal(cls, position, speed, effort, wait=True):
        goal = RobotGripGoal()
        goal.goal = cls.pack_gripper_msg(position,speed,effort)
        goal.wait_for_finish = wait
        return goal

    @classmethod
    def pack_gripper_msg(cls, position, speed, effort):
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
        return msg