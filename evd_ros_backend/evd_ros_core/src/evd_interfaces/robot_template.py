'''
Robot template defines an abstracted single-arm with single gripper robot 
that interacts with EvD's robot_interface. It is up to the designer to specifiy the
callback functions used in this template for their robot.

Basic robot behavior to move to a joint state or a pose is assumed. Gripper with
position (and optional speed/effort) is assumed.

Base robot is conceptualized around a UR3e with Robotiq 85 2-Finger gripper.

The asynchronous methods use the ACK/NACK as feedback channel whereas the synchronous
methods use services. All robot behavior is also being presented in a generalized 
status update pushed from this node. Fields to update status are provided in this
template.
'''
import rospy

from geometry_msgs.msg import Pose
from evd_ros_core.srv import SetRobotMove, SetRobotMoveResponse
from evd_ros_core.srv import SetRobotMoveTrajectory, SetRobotMoveTrajectoryResponse
from evd_ros_core.srv import SetRobotGrip, SetRobotGripResponse
from evd_ros_core.msg import RobotAck, RobotStatus, RobotStop, RobotPause, RobotMove, RobotMoveTrajectory, RobotGrip, RobotInitialize


class RobotTemplate:

    VALID_STATES = [
        RobotStatus.STATUS_RUNNING,
        RobotStatus.STATUS_IDLE,
        RobotStatus.STATUS_PAUSED,
        RobotStatus.STATUS_ERROR
    ]

    def __init__(self, prefix=None, real_robot=False, init_fnt=None, estop_fnt=None, pause_fnt=None, 
                 move_fnt=None, move_traj_fnt=None, grip_fnt=None):
        self._prefix = prefix
        prefix_fmt = prefix+'/' if prefix != None else ''

        self._init_fnt = init_fnt
        self._estop_fnt = estop_fnt
        self._pause_fnt = pause_fnt
        self._move_fnt = move_fnt
        self._move_traj_fnt = move_traj_fnt
        self._grip_fnt = grip_fnt

        self._current_pose = Pose()
        self._current_joints = []
        self._current_gripper_position = -1
        self._is_real_robot = real_robot
        self._is_in_emergency_stop = False
        self._is_running = False
        self._gripper_object_detected = False
        self._status = RobotStatus.STATUS_IDLE

        self.ack_pub = rospy.Publisher('{0}robot/ack'.format(prefix_fmt), RobotAck, queue_size=10)
        self.status_pub = rospy.Publisher('{0}robot/wait'.format(prefix_fmt), RobotStatus, queue_size=10)

        self.init_sub = rospy.Subscriber('{0}robot/initialize'.format(prefix_fmt),RobotInitialize, self._init_cb)
        self.estop_sub = rospy.Subscriber('{0}robot/estop'.format(prefix_fmt), RobotStop, self._estop_cb)
        self.pause_sub = rospy.Subscriber('{0}robot/pause'.format(prefix_fmt), RobotPause, self._pause_cb)
        self.move_sub = rospy.Subscriber('{0}robot/move'.format(prefix_fmt), RobotMove, self._move_async_cb)
        self.move_trajectory_sub = rospy.Subscriber('{0}robot/move_trajectory'.format(prefix_fmt), RobotMoveTrajectory, self._move_trajectory_async_cb)
        self.grip_sub = rospy.Subscriber('{0}robot/grip'.format(prefix_fmt), RobotGrip, self._grip_async_cb)

        self.move_srv = rospy.Service('{0}robot/set_move',SetRobotMove, self._move_sync_cb)
        self.move_trajectory_srv = rospy.Service('{0}robot/set_move_trajectory',SetRobotMoveTrajectory, self._move_trajectory_sync_cb)
        self.grip_srv = rospy.Service('{0}robot/set_grip',SetRobotGrip, self._grip_sync_cb)

    def _init_cb(self, msg):
        ack = False

        set_gripper = msg.gripper_position != -1
        set_arm = len(msg.arm_joints) > 0

        if self._init_fnt != None:
            ack = self._init_fnt(
                set_gripper = set_gripper,
                gripper_position=msg.gripper_position,
                set_arm = set_arm,
                arm_joints=msg.arm_joints)
        
        if set_arm:
            retmsg = RobotAck('arm',ack)
            self.ack_pub.publish(retmsg)

        if set_gripper:
            retmsg = RobotAck('grip',ack)
            self.ack_pub.publish(retmsg)

    def _estop_cb(self, msg):
        ack = False
        if self._estop_fnt != None:
            ack = self._estop_fnt(msg.emergency) == True
        
        retmsg = RobotAck('arm',ack)
        self.ack_pub.publish(retmsg)

    def _pause_cb(self, msg):
        ack = False
        if self._pause_fnt != None:
            ack = self._pause_fnt(msg.state) == True
        
        retmsg = RobotAck('arm',ack)
        self.ack_pub.publish(retmsg)

    def _move_async_cb(self, msg):
        ack = False
        if self.is_running and self._move_fnt != None:
            ack = self._move_fnt(msg) == True
        
        retmsg = RobotAck('arm',ack)
        self.ack_pub.publish(retmsg)

    def _move_sync_cb(self, request):
        status = False
        if self.is_running and self._move_fnt != None:
            status = self._move_fnt(request.goal) == True

        response = SetRobotMoveResponse()
        response.status = status
        return response

    def _move_trajectory_async_cb(self, msg):
        ack = False
        if self.is_running and self._move_traj_fnt != None:
            ack = self._move_traj_fnt(msg.moves) == True
        
        retmsg = RobotAck('arm',ack)
        self.ack_pub.publish(retmsg)

    def _move_trajectory_sync_cb(self, request):
        status = False
        if self.is_running and self._move_traj_fnt != None:
            status = self._move_traj_fnt(request.goal.moves) == True
        
        response = SetRobotMoveTrajectoryResponse()
        response.status = status
        return response

    def _grip_async_cb(self, msg):
        ack = False
        if self.is_running and self._grip_fnt != None:
            ack = self._grip_fnt(msg) == True
        
        retmsg = RobotAck('grip',ack)
        self.ack_pub.publish(retmsg)

    def _grip_sync_cb(self, request):
        status = False
        if self.is_running and self._grip_fnt != None:
            status = self._grip_fnt(request.goal) == True
        
        response = SetRobotGripResponse()
        response.status = status
        return response

    @property
    def current_pose(self):
        return self._current_pose

    @current_pose.setter
    def current_pose(self, value):
        self._current_pose = value

    @property
    def current_joints(self):
        return self._current_joints

    @current_joints.setter
    def current_joints(self, value):
        self._current_joints = value

    @property
    def current_gripper_position(self):
        return self._current_gripper_position

    @current_gripper_position.setter
    def current_gripper_position(self, value):
        self._current_gripper_position = value

    @property
    def is_real_robot(self):
        return self._is_real_robot

    @is_real_robot.setter
    def is_real_robot(self, value):
        self._is_real_robot = value

    @property
    def is_in_emergency_stop(self):
        return self._is_in_emergency_stop
    
    @is_in_emergency_stop.setter
    def is_in_emergency_stop(self, value):
        self._is_in_emergency_stop = value

    @property
    def is_running(self):
        return self._is_running

    @is_running.setter
    def is_running(self, value):
        self._is_running = value

    @property
    def gripper_object_detected(self):
        return self._gripper_object_detected

    @gripper_object_detected.setter
    def gripper_object_detected(self, value):
        self._gripper_object_detected = value

    @property
    def status(self):
        return self._status

    @status.setter
    def status(self, value):
        if not value in self.VALID_STATES:
            raise Exception('Status must be a valid state')

        self._status = value

    def update_status(self):
        msg = RobotStatus()
        msg.arm_pose = self._current_pose
        msg.arm_joints = self._current_joints
        msg.gripper_position = self._current_gripper_position
        msg.is_real_robot = self._is_real_robot
        msg.is_in_emergency_stop = self._is_in_emergency_stop
        msg.is_running = self._is_running
        msg.gripper_object_detected = self._gripper_object_detected
        msg.status = self._status

        self.status_pub.publish(msg)