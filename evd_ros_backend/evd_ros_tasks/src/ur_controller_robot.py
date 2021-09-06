#!/usr/bin/env python3

'''
Implements a UR Controller middleware that exposed URScript behavior.
This version is compatible with RobotInterace defined in evd_ros_core.
Additionally, freedrive and servoing behavior exposed.

NOTE this has not been ran / tested since converting to the new RobotTemplate
Some work is needed to get this fully functional.
'''

import tf
import time
import math
import rospy

from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from ur_msgs.msg import RobotModeDataMsg
from robotiq_85_msgs.msg import GripperCmd, GripperStat
from geometry_msgs.msg import Pose, Point, Quaternion
from evd_ros_core.msg import RobotMove, RobotStatus, RobotGrip

from evd_interfaces.robot_template import RobotTemplate
from evd_interfaces.frontend_interface import FrontendInterface
from evd_script import Position, ReachSphere, PinchPoint, CollisionMesh, OccupancyZone


# Time Scaling Servoing Constants 
MIN_M = -30.0
MAX_M = 1.0
MIN_TIME = 0.05
MAX_TIME = 0.9

# Default Movement Constants
DEFAULT_GAIN = 100
DEFAULT_LOOKAHEAD = 0.1
DEFAULT_TIME_SCALARS = [100,100,100,100,100,100]
DEFAULT_TIMESTEP = 0.12
DEFAULT_JOINT_ACCELERATION = 1.4
DEFAULT_LINEAR_ACCELERATION = 1.2


class URController(RobotTemplate):

    def __init__(self, prefix=None, real_robot=False, rate=5, end_effector_link='ee_link', base_link='base_link'):
        super(URController,self).__init__(
            prefix, real_robot,
            init_fnt=self.initialize,
            estop_fnt=self.estop,
            pause_fnt=self.pause,
            move_fnt=self.move,
            move_traj_fnt=self.move_trajectory,
            grip_fnt=self.grip)

        self._reach_sphere = ReachSphere(0.8, offset=Position(0,0,0.15))
        self._pinch_points = [
            PinchPoint(link='simulated_shoulder_link', radius=0.075, length=0.2, offset=Position.from_axis('z',-0.05)),
            PinchPoint(link='simulated_upper_arm_link', radius=0.075, length=0.2, offset=Position.from_axis('z',0.075)),
            PinchPoint(link='simulated_forearm_link', radius=0.075, length=0.2, offset=Position.from_axis('z',0.075)),
            PinchPoint(link='simulated_wrist_1_link', radius=0.06, length=0.17, offset=Position.from_axis('z',-0.05)),
            PinchPoint(link='simulated_wrist_3_link', radius=0.1, length=0.16, offset=Position.from_axis('z',0.1))
        ]
        self._collision_meshes = [
            CollisionMesh(link='pedestal_link', mesh_id='package://evd_ros_tasks/description/meshes/collision/Pedestal.stl'),
        ]
        self._occupancy_zones = [
            OccupancyZone(OccupancyZone.ROBOT_TYPE, sclX=1.6, sclZ=1.2, height=-0.77)
        ]

        self._program = FrontendInterface(use_registration=True, register_cb=self._call_to_register)

        self._last_js_msg = JointState()
        self._last_js_msg_time = 0
        self._last_rms_msg = RobotModeDataMsg()
        self._last_rms_msg_time = 0
        self._last_gstat_msg = GripperStat()
        self._last_gstat_msg_time = 0

        prefix_fmt = prefix+'/' if prefix != None else ''

        self._rate = rate
        self._ee_link = end_effector_link
        self._base_link = base_link

        self._urscript_pub = rospy.Publisher('ur_hardware_interface/script_command',String,queue_size=10)
        self._gripper_cmd_pub = rospy.Publisher("gripper/cmd", GripperCmd, queue_size=10)

        self._joint_state_sub = rospy.Subscriber('{0}joint_states'.format(prefix_fmt),JointState,self._joint_state_cb)
        self._robot_mode_state_sub = rospy.Subscriber('{0}ur_driver/robot_mode_state'.format(prefix_fmt),RobotModeDataMsg,self._robot_mode_state_cb)
        self._gripper_stat_sub = rospy.Publisher('{0}gripper/stat'.format(prefix_fmt), GripperStat, self._gripper_stat_cb, queue_size=10)
        self._tf_listener = tf.TransformListener()
        
        self._freedrive_sub = rospy.Subscriber('ur_controller/freedrive',Bool,self._freedrive_cb)
        self._servoing_sub = rospy.Subscriber('ur_controller/servoing',RobotMove,self._servoing_cb)

    def _call_to_register(self):
        dct_list = []

        dct_list.append(self._reach_sphere.to_dct())
        dct_list.extend([p.to_dct() for p in self._pinch_points])
        dct_list.extend([c.to_dct() for c in self._collision_meshes])
        dct_list.extend([o.to_dct() for o in self._occupancy_zones])

        self._program.register(dct_list)

        print('robot-registered')

    def spin(self):
        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():

            # Update end-effector being tracked by interface
            try:
                ((tx,ty,tz), (rx,ry,rz,rw)) = self._tf_listener.lookupTransform(self._ee_link,self._base_link,rospy.Time(0))
                pose = Pose(Point(tx,ty,tz),Quaternion(rx,ry,rz,rw))
                self.current_pose = pose
            except:
                pass

            # Push periodic updates
            self.update_status()
            rate.sleep()

    '''
    Low-Level Hardware callbacks
    '''

    def _joint_state_cb(self, msg):
        self._last_js_msg = msg
        self._last_js_msg_time = time.time()
        self.current_joints = msg.position

    def _robot_mode_state_cb(self, msg):
        self._last_rms_msg = msg
        self._last_rms_msg_time = time.time()
        self.is_in_emergency_stop = msg.is_emergency_stopped or msg.is_protective_stopped
        self.is_real_robot = msg.is_robot_connected and msg.is_real_robot_enabled

    def _gripper_stat_cb(self, msg):
        self._last_gstat_msg = msg
        self._last_gstat_msg_time = time.time()
        self.current_gripper_position = msg.position
        self.gripper_object_detected = msg.obj_detected

    '''
    UR Controller Specific Behavior
    '''

    def _freedrive_cb(self, msg):
        self.is_running = msg.data

        cmd =  'def prog():\n'
        if msg.data:
            self.status = RobotStatus.STATUS_RUNNING

            cmd += '\twhile (True):\n'
            cmd += '\t\tfreedrive_mode()\n'
            cmd += '\t\tsync()\n'
            cmd += '\tend\n'
        else:
            self.status = RobotStatus.STATUS_IDLE
            cmd += '\tend_freedrive_mode()\n'
        cmd += 'end\n'
        self._urscript_pub.publish(String(cmd))

    def _servoing_cb(self, msg):
        # a servo op is "instantanous" and interruptable, hence considered idle
        self.status = RobotStatus.STATUS_IDLE
        self.is_running = False 

        if msg.motion_type == RobotMove.EE_IK:
            cmd = URController.pack_servoing_joint_pose_cmd(msg.target_pose,msg.velocity,DEFAULT_LINEAR_ACCELERATION)
        else:
            cmd = URController.pack_servoing_joint_joints_cmd(msg.target_joints,msg.velocity,DEFAULT_JOINT_ACCELERATION)

        self._urscript_pub.publish(String(cmd))

    '''
    Robot Template Behavior
    '''

    def initialize(self, set_gripper=False, gripper_position=None, set_arm=False, arm_joints=None):
        # Behavior is not instantaneous. Must jog robot to position.
        grip_status = True
        if set_gripper:
            msg = RobotGrip()
            msg.position = gripper_position
            grip_status = self.grip(msg)

        arm_status = True
        if set_arm:
            msg = RobotMove()
            msg.motion_type = RobotMove.JOINT
            msg.target_joints = arm_joints
            msg.velocity = RobotMove.JOINT_VELOCITY
            arm_status = self.move(msg)

        return grip_status and arm_status # ACK / NACK

    def estop(self, emergency):
        self.status = RobotStatus.STATUS_ERROR if emergency else RobotStatus.STATUS_IDLE
        self.is_running = False

        if emergency:
            cmd = URController.pack_stop_joint_cmd(DEFAULT_JOINT_ACCELERATION)
        else:
            cmd = URController.pack_stop_linear_cmd(DEFAULT_LINEAR_ACCELERATION)
        self._urscript_pub.publish(String(cmd))
    
        self._running_gripper = False
        cmd = URController.pack_gripper_stop_cmd()
        self._gripper_cmd_pub.publish(cmd)

        return True # ACK
            
    def pause(self, state):
        # Pause is not supported on the real robot at this point!
        if self.is_running:
            self.estop(True)

        return False #NACK

    def move(self, msg): #RobotMove
        self.status = RobotStatus.STATUS_RUNNING
        self.is_running = True

        if msg.motion_type == RobotMove.EE_IK:
            cmd = URController.pack_servoing_joint_pose_cmd(msg.target_pose,msg.velocity,DEFAULT_LINEAR_ACCELERATION)
        else:
            cmd = URController.pack_servoing_joint_joints_cmd(msg.target_joints,msg.velocity,DEFAULT_JOINT_ACCELERATION)

        # publish to robot & wait for movement to complete
        self._urscript_pub.publish(String(cmd))
        self._execute_wait('arm')

        self.status = RobotStatus.STATUS_IDLE
        self.is_running = False
        return True #ACK

    def move_trajectory(self, moves): #RobotMove[]
        status = True

        #generate urscript program for path
        cmd = "def prog():\n"
        for move in moves:
            if move.motion_type == RobotMove.EE_IK:
                cmd += self.__move_linear(move)
            elif move.motion_type == RobotMove.JOINT:
                cmd += self.__move_joint_joints(move)
            else:
                status = False
        cmd += "end\n"

        if not status:
            # could not properly pack trajectory
            return False #NACK
        elif len(moves) == 0:
            # Empty list can be run in zero time :)
            return True #ACK

        # publish to robot & wait for movement to complete
        self._urscript_pub.publish(String(cmd))
        self._execute_wait('arm')

        return True #ACK

    def grip(self, msg): #RobotGrip
        self.status = RobotStatus.STATUS_RUNNING
        self.is_running = True

        cmd = URController.pack_gripper_move_cmd(msg.position,msg.speed,msg.effort)
        self._gripper_cmd_pub.publish(cmd)

        self._execute_wait('grip')

        self.status = RobotStatus.STATUS_IDLE
        self.is_running = False
        return True #ACK

    def _execute_wait(self, arm_or_grip='arm'):
        state = 'start'
        init_time = time.time()
        while True:
            
            # check if forced to stop
            if not self.is_running:
                state = 'stopped'
                break

            # get running status
            if arm_or_grip == 'arm':
                running = self._last_rms_msg.is_program_running
            else:
                running = self._last_gstat_msg.is_moving
            
            # run state machine to confirm behavior
            if state == 'start':
                if running:
                    state = 'moving'
                    continue

                elif time.time() - init_time >= 0.2:
                    # Program never ran, assume it is already at position
                    state = 'timeout'
                    break
            
            elif state == 'moving':
                # while controller says it is running, stay in this loop
                if not running:
                    state = 'stopped'
                    break

            rospy.sleep(0.1)

    '''
    Hardware Level - State Properties
    '''

    def get_last_gripper_state(self):
        return self._last_gstat_msg, self._last_gstat_msg_time

    def get_last_robot_mode_state(self):
        return self._last_rms_msg, self._last_rms_msg_time

    def get_last_joint_state(self):
        return self._last_js_msg, self._last_js_msg_time

    '''
    Class Methods
        - Used to pack commands
    '''

    @classmethod
    def pack_move_circular_cmd(self, target_pose, path_pose, acceleration, velocity, radius, circular_constrained):
        px, py, pz, rx, ry, rz = self.unpack_formatted_pose(target_pose)
        ppx, ppy, ppz, prx, pry, prz = self.unpack_formatted_pose(path_pose)

        cmd =  "\trv = rpy2rotvec([{0},{1},{2}])\n".format(rx,ry,rz)
        cmd += "\tpose_target = p[{0},{1},{2},rv[0],rv[1],rv[2]]\n".format(px,py,pz)
        cmd += "\trv = rpy2rotvec([{0},{1},{2}])\n".format(prx,pry,prz)
        cmd += "\tpath_point = p[{0},{1},{2},rv[0],rv[1],rv[2]]\n".format(ppx,ppy,ppz)
        cmd += "\tmovec(path_point,pose_target,{0},{1},{2},{3})\n".format(acceleration,velocity,radius,int(circular_constrained))
        return cmd

    @classmethod
    def pack_move_joint_pose_cmd(self, pose, acceleration, velocity, time, radius):
        px, py, pz, rx, ry, rz = self.unpack_formatted_pose(pose)

        cmd =  "\trv = rpy2rotvec([{0},{1},{2}])\n".format(rx,ry,rz)
        cmd += "\tpose_target = p[{0},{1},{2},rv[0],rv[1],rv[2]]\n".format(px,py,pz)
        cmd += "\tmovej(pose_target,{0},{1},{2},{3})\n".format(acceleration,velocity,time,radius)
        return cmd

    @classmethod
    def pack_move_joint_joints_cmd(self, js, acceleration, velocity, time, radius):
        cmd =  "\tjoints = [{0},{1},{2},{3},{4},{5}]\n".format(js[0],js[1],js[2],js[3],js[4],js[5])
        cmd += "\tmovej(joints,{0},{1},{2},{3})\n".format(acceleration,velocity,time,radius)
        return cmd

    @classmethod
    def pack_move_linear_cmd(cls, pose, acceleration, velocity, time, radius):
        px, py, pz, rx, ry, rz = cls.unpack_formatted_pose(pose)

        cmd =  "\trv = rpy2rotvec([{0},{1},{2}])\n".format(rx,ry,rz)
        cmd += "\tpose_target = p[{0},{1},{2},rv[0],rv[1],rv[2]]\n".format(px,py,pz)
        cmd += "\tmovel(pose_target,{0},{1},{2},{3})\n".format(acceleration,velocity,time,radius)
        return cmd

    @classmethod
    def pack_move_process_cmd(cls, pose, acceleration, velocity, radius):
        px, py, pz, rx, ry, rz = cls.unpack_formatted_pose(pose)

        cmd =  "\trv = rpy2rotvec([{0},{1},{2}])\n".format(rx,ry,rz)
        cmd += "\tpose_target = p[{0},{1},{2},rv[0],rv[1],rv[2]]\n".format(px,py,pz)
        cmd += "\tmovep(pose_target,{0},{1},{2})\n".format(acceleration,velocity,radius)
        return cmd

    @classmethod
    def pack_servoing_joint_joints_cmd(cls, js, velocity, acceleration, lookahead_time, gain, time_scalars=None, prev_js=None, timestep=None):
        # Time scaling attempts to provide a dynamic timestep based on the difference in prev to current joint states. This can stabilize some
        # of the jerky (higher-order overshoot) behavior when servoing aggressively.
        if time_scalars != None:
            j = max([time_scalars[i] * abs(js[i] - prev_js[i]) for i in range(0,len(time_scalars))])
            if j > 0:
                m = math.log(j)
                if m <= MIN_M:
                    time = MIN_TIME
                elif m >= MAX_M:
                    time = MAX_TIME
                else:
                    time = (MAX_TIME - MIN_TIME) / (MAX_M - MIN_M) * (m - MIN_M) + MIN_TIME
            else:
                time = MIN_TIME
        else:
            time = timestep if timestep != None else DEFAULT_TIMESTEP

        cmd =  "def prog():\n"
        cmd += "\tjoints = [{0},{1},{2},{3},{4},{5}]\n".format(js[0],js[1],js[2],js[3],js[4],js[5])
        cmd += "\tservoj(joints,{0},{1},{2},{3},{4})\n".format(acceleration,velocity,time,lookahead_time,gain)
        cmd += "end\n"
        return cmd

    @classmethod
    def pack_servoing_joint_pose_cmd(cls, pose, velocity, acceleration, timestep, lookahead_time, gain):
        px, py, pz, rx, ry, rz = cls.unpack_formatted_pose(pose)

        cmd =  "def prog():\n"
        cmd += "\trv = rpy2rotvec([{0},{1},{2}])\n".format(rx,ry,rz)
        cmd += "\tpose_target = p[{0},{1},{2},rv[0],rv[1],rv[2]]\n".format(px,py,pz)
        cmd += "\tjoints = get_inverse_kin(pose_target)\n"
        cmd += "\tservoj(joints,{0},{1},{2},{3},{4})\n".format(acceleration,velocity,timestep,lookahead_time,gain)
        cmd += "end\n"
        return cmd

    @classmethod
    def pack_servoing_circular_cmd(cls, pose, radius, velocity, acceleration):
        px, py, pz, rx, ry, rz = cls.unpack_formatted_pose(pose)

        cmd =  "def prog():\n"
        cmd += "\trv = rpy2rotvec([{0},{1},{2}])\n".format(rx,ry,rz)
        cmd += "\tpose_target = p[{0},{1},{2},rv[0],rv[1],rv[2]]\n".format(px,py,pz)
        cmd += "\tservoc(pose_target,{0},{1},{2})\n".format(acceleration,velocity,radius)
        cmd += "end\n"
        return cmd

    @classmethod
    def unpack_formatted_pose(cls, pose):
        px = -pose.position.x
        py = -pose.position.y
        pz = pose.position.z
        (rx, ry, rz) = tf.transformations.euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w],
            'sxyz')
        return px, py, pz, rx, ry, rz

    @classmethod
    def pack_stop_joint_cmd(cls, acceleration):
        return "stopj({0})\n".format(acceleration)

    @classmethod
    def pack_stop_linear_cmd(cls, acceleration):
        return "stopl({0})\n".format(acceleration)

    @classmethod
    def pack_gripper_stop_cmd(cls):
        msg = GripperCmd()
        msg.stop = True
        return msg

    @classmethod
    def pack_gripper_move_cmd(cls, position, speed, force):
        msg = GripperCmd()
        msg.position = position
        msg.speed = speed
        msg.force = force
        return msg


if __name__ == "__main__":
    rospy.init_node('ur_controller_robot')

    prefix = rospy.get_param('prefix',None)
    rate = rospy.get_param('rate',5)

    node = URController(prefix,rate)
    node.spin()