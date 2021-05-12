#!/usr/bin/env python3

'''
Convert ur_mode and gripper_stat to a consistent interface for robot_interface
'''

import tf
import time
import math
import rospy
import actionlib

from std_msgs.msg import Bool, String
from ur_msgs.msg import RobotModeDataMsg
from evd_ros_core.msg import Stop, Servo, Move, Grip
from robotiq_85_msgs.msg import GripperCmd, GripperStat
from geometry_msgs.msg import Pose, Quaternion, Vector3
from evd_ros_core.msg import MoveTrajectoryAction, MoveTrajectoryGoal, MoveTrajectoryResult, MoveTrajectoryFeedback


MIN_M = -30.0
MAX_M = 1.0
MIN_TIME = 0.05
MAX_TIME = 0.9


class URController:

    def __init__(self, gain, lookahead, time_scalars, timestep):
        self._gain = gain
        self._timestep = timestep
        self._lookahead = lookahead
        self._running_trajectory = False
        self._time_scalars = time_scalars
        self._robot_state = RobotModeDataMsg()
        self._gripper_state = GripperStat()

        self._urscript_pub = rospy.Publisher('ur_hardware_interface/script_command',String,queue_size=10)
        self._gripper_cmd_pub = rospy.Publisher("gripper/cmd", GripperCmd, queue_size=10)
        self._gripper_stat_sub = rospy.Publisher('gripper/stat', GripperStat, self._gripper_stat_cb, queue_size=10)

        self._ur_mode_sub = rospy.Subscriber('ur_driver/robot_mode_state',RobotModeDataMsg,self._ur_mode_cb)
        self._freedrive_sub = rospy.Subscriber('robot_control/freedrive',Bool,self._freedrive_cb)
        self._servoing_sub = rospy.Subscriber('robot_control/servoing',Servo,self._servoing_cb)
        self._stop_sub = rospy.Subscriber('robot_control/stop',Stop,self._stop_cb)
        self._gripper_sub = rospy.Subscriber('robot_control/gripper',Grip,self._gripper_cb)

        self._move_trajectory_as = actionlib.SimpleActionServer('robot_control/move_trajectory',MoveTrajectoryAction,execute_cb=self._move_trajectory_cb,auto_start=False)
        self._move_trajectory_as.start()

    def _ur_mode_cb(self, msg):
        self._robot_state = msg

    def _freedrive_cb(self, msg):
        cmd =  'def prog():\n'
        if msg.data:
            cmd += '\twhile (True):\n'
            cmd += '\t\tfreedrive_mode()\n'
            cmd += '\t\tsync()\n'
            cmd += '\tend\n'
        else:
            cmd += '\tend_freedrive_mode()\n'
        cmd += 'end\n'
        self._urscript_pub.publish(String(cmd))

    def _servoing_cb(self, msg):
        self._running_trajectory = False

        cmd = ''
        if msg.motion_type == Servo.CIRCULAR:
            cmd = self.__servoing_circular(msg.target_pose,msg.radius,msg.velocity,msg.acceleration)
        elif msg.motion_type == Servo.JOINT:
            if msg.use_ur_ik:
                cmd = self.__servoing_joint_pose(msg.target_pose,msg.velocity,msg.acceleration)
            else:
                cmd = self.__servoing_joint_joints(msg.target_joints,msg.velocity,msg.acceleration)

        self._urscript_pub.publish(String(cmd))

    def __servoing_joint_joints(self, js, velocity, acceleration):
        if self._time_scalars != None:
            j = max([self._time_scalars[i] * abs(js[i] - self._js[i]) for i in range(0,len(self._ordering))])
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
            time = self._timestep
        self._js = js

        cmd =  "def prog():\n"
        cmd += "\tjoints = [{0},{1},{2},{3},{4},{5}]\n".format(js[0],js[1],js[2],js[3],js[4],js[5])
        cmd += "\tservoj(joints,{0},{1},{2},{3},{4})\n".format(acceleration,velocity,time,self._lookahead_time,self._gain)
        cmd += "end\n"
        return cmd

    def __servoing_joint_pose(self, pose, velocity, acceleration):
        px, py, pz, rx, ry, rz = self.___format_pose(pose)

        cmd =  "def prog():\n"
        cmd += "\trv = rpy2rotvec([{0},{1},{2}])\n".format(rx,ry,rz)
        cmd += "\tpose_target = p[{0},{1},{2},rv[0],rv[1],rv[2]]\n".format(px,py,pz)
        cmd += "\tjoints = get_inverse_kin(pose_target)\n"
        cmd += "\tservoj(joints,{0},{1},{2},{3},{4})\n".format(acceleration,velocity,self._timestep,self._lookahead_time,self._gain)
        cmd += "end\n"
        return cmd

    def __servoing_circular(self, pose, radius, velocity, acceleration):
        px, py, pz, rx, ry, rz = self.___format_pose(pose)

        cmd =  "def prog():\n"
        cmd += "\trv = rpy2rotvec([{0},{1},{2}])\n".format(rx,ry,rz)
        cmd += "\tpose_target = p[{0},{1},{2},rv[0],rv[1],rv[2]]\n".format(px,py,pz)
        cmd += "\tservoc(pose_target,{0},{1},{2})\n".format(acceleration,velocity,radius)
        cmd += "end\n"
        return cmd

    def ___format_pose(self, pose):
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

    def _move_trajectory_cb(self, goal):
        self._running_trajectory = True

        result = MoveTrajectoryResult()
        result.status = True
        result.message = ''

        feedback = MoveTrajectoryFeedback()
        feedback.message = 'starting action'
        self._move_trajectory_as.publish_feedback(feedback)

        #generate urscript program for path
        cmd = "def prog():\n"
        for move in goal.moves:
            if move.motion_type == Move.CIRCULAR:
                cmd += self.__move_circular(move)
            elif move.motion_type == Move.JOINT:
                if move.use_ur_ik:
                    cmd += self.__move_joint_pose(move)
                else:
                    cmd += self.__move_joint_joints(move)
            elif move.motion_type == Move.LINEAR:
                cmd += self.__move_linear(move)
            elif move.motion_type == Move.PROCESS:
                cmd += self.__move_process(move)
            else:
                result.status = False
                result.message = 'invalid motion type encountered'
        cmd += "end\n"

        # early stopping on error or if zero items
        if not result.status or len(goal.moves) == 0:
            self._move_trajectory_as.set_succeeded(result)

        feedback = MoveTrajectoryFeedback()
        feedback.message = 'generated urscript'
        self._move_trajectory_as.publish_feedback(feedback)

        # publish to robot
        self._urscript_pub.publish(String(cmd))

        feedback = MoveTrajectoryFeedback()
        feedback.message = 'published urscript'
        self._move_trajectory_as.publish_feedback(feedback)

        # wait for movement to complete
        if goal.wait_for_finish:
            state = 'start'
            init_time = time.time()
            while True:
                # check if forced to stop
                if not self._running_trajectory:
                    state = 'stopped'

                    self._urscript_pub.publish(String("stopj({0})\n".format(1.2)))

                    result.status = False
                    result.message = 'forced to stop in node'
                    self._move_trajectory_as.set_succeeded(result)
                    break

                elif self._move_trajectory_as.is_preempt_requested():
                    state = 'preempted'

                    self._urscript_pub.publish(String("stopj({0})\n".format(1.2)))

                    result.status = False
                    result.message = 'preempted'
                    self._server.set_preempted(result)
                    break

                # run state machine
                if state == 'start':
                    if self._robot_state.is_program_running:
                        state = 'moving'
                        continue

                    elif time.time() - init_time >= 0.2:
                        state = 'timeout'

                        result.status = True
                        result.message = 'Robot program never ran, could already be at state'
                        self._move_trajectory_as.set_succeeded(result)
                        break

                elif state == 'moving':
                    if not self._robot_state.is_program_running:
                        state = 'stopped'

                        result.status = True
                        result.message = 'Trajectory finished executing'
                        self._move_trajectory_as.set_succeeded(result)
                        break

                # waiting timestep
                rospy.sleep(0.1)
        else:
            result.status = True
            result.message = 'Instructed to ignore trajectory execution state'
            self._move_trajectory_as.set_succeeded(result)

    def __move_circular(self, msg):
        px, py, pz, rx, ry, rz = self.___format_pose(msg.target_pose)
        ppx, ppy, ppz, prx, pry, prz = self.___format_pose(msg.path_pose)

        cmd =  "\trv = rpy2rotvec([{0},{1},{2}])\n".format(rx,ry,rz)
        cmd += "\tpose_target = p[{0},{1},{2},rv[0],rv[1],rv[2]]\n".format(px,py,pz)
        cmd += "\trv = rpy2rotvec([{0},{1},{2}])\n".format(prx,pry,prz)
        cmd += "\tpath_point = p[{0},{1},{2},rv[0],rv[1],rv[2]]\n".format(ppx,ppy,ppz)
        cmd += "\tmovec(path_point,pose_target,{0},{1},{2},{3})\n".format(msg.acceleration,msg.velocity,msg.radius,int(msg.circular_constrained))
        return cmd

    def __move_joint_pose(self, msg):
        px, py, pz, rx, ry, rz = self.___format_pose(msg.target_pose)

        cmd =  "\trv = rpy2rotvec([{0},{1},{2}])\n".format(rx,ry,rz)
        cmd += "\tpose_target = p[{0},{1},{2},rv[0],rv[1],rv[2]]\n".format(px,py,pz)
        cmd += "\tmovej(pose_target,{0},{1},{2},{3})\n".format(msg.acceleration,msg.velocity,msg.time,msg.radius)
        return cmd

    def __move_joint_joints(self, msg):
        js = msg.target_joints

        cmd =  "\tjoints = [{0},{1},{2},{3},{4},{5}]\n".format(js[0],js[1],js[2],js[3],js[4],js[5])
        cmd += "\tmovej(joints,{0},{1},{2},{3})\n".format(msg.acceleration,msg.velocity,msg.time,msg.radius)
        return cmd

    def __move_linear(self, msg):
        px, py, pz, rx, ry, rz = self.___format_pose(msg.target_pose)

        cmd =  "\trv = rpy2rotvec([{0},{1},{2}])\n".format(rx,ry,rz)
        cmd += "\tpose_target = p[{0},{1},{2},rv[0],rv[1],rv[2]]\n".format(px,py,pz)
        cmd += "\tmovel(pose_target,{0},{1},{2},{3})\n".format(msg.acceleration,msg.velocity,msg.time,msg.radius)
        return cmd

    def __move_process(self, msg):
        px, py, pz, rx, ry, rz = self.___format_pose(msg.target_pose)

        cmd =  "\trv = rpy2rotvec([{0},{1},{2}])\n".format(rx,ry,rz)
        cmd += "\tpose_target = p[{0},{1},{2},rv[0],rv[1],rv[2]]\n".format(px,py,pz)
        cmd += "\tmovep(pose_target,{0},{1},{2},{3})\n".format(msg.acceleration,msg.velocity,msg.radius)
        return cmd

    def _stop_cb(self, msg):

        if msg.motion_type == Stop.JOINT:
            cmd = "stopj({0})\n".format(msg.acceleration)
        else: # default to this case for safety
            cmd = "stopl({0})\n".format(msg.acceleration)

        self._urscript_pub.publish(String(cmd))
        self._running_trajectory = False

    def _gripper_cb(self, msg):
        cmd = GripperCmd()
        cmd.position = msg.position / 100 * 0.085
        cmd.speed = msg.speed / 100
        cmd.force = msg.effort / 100
        self._gripper_cmd_pub.publish(cmd)

    def _gripper_stat_cb(self, msg):
        self._gripper_state = msg


if __name__ == "__main__":
    rospy.init_node('ur_controller')

    gain = rospy.get_param('~gain')
    lookahead = rospy.get_param('~lookahead')
    time_scalars = rospy.get_param('~time_scalars',None)
    timestep = rospy.get_param('~timestep',MIN_TIME)

    node = URController(gain,lookahead,time_scalars,timestep)
    rospy.spin()
