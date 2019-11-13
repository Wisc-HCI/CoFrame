#!/usr/bin/env python

import tf
import math
import rospy
import actionlib

from std_msgs.msg import Bool
from pyquaternion import Quaternion
from cobots_core.msg import Stop, Servo, Move
from cobots_core.msg import MoveTrajectoryAction, MoveTrajectoryGoal, MoveTrajectoryResult, MoveTrajectoryFeedback
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommandFeedback, GripperCommandResult

MIN_M = -30.0
MAX_M = 1.0
MIN_TIME = 0.05
MAX_TIME = 0.9

DEFAULT_STEADY_STATE_LENGTH = 3
DEFAULT_STEADY_STATE_THRESHOLD = 0.01


class URController:

    def __init__(self, ee_frame, base_frame, gain, lookahead, time_scalars,
                 timestep, steady_state_length, steady_state_threshold):
        self._gain = gain
        self._ee_frame = ee_frame
        self._timestep = timestep
        self._lookahead = lookahead
        self._base_frame = base_frame
        self._running_trajectory = False
        self._time_scalars = time_scalars
        self._steady_state_length = steady_state_length
        self._steady_state_threshold = steady_state_threshold

        self._tf_listener = tf.TransformListener()

        self._urscript_pub = rospy.Publisher('ur_driver/URScript',String,queue_size=10)

        self._freedrive_sub = rospy.Subscriber('robot_control/freedrive',Bool,self._freedrive_cb)
        self._servoing_sub = rospy.Subscriber('robot_control/servoing',Servo,self._servoing_cb)
        self._stop_sub = rospy.Subscriber('robot_control/stop',Stop,self._stop_cb)

        self._move_trajectory_as = actionlib.SimpleActionServer('robot_control/move_trajectory',MoveTrajectoryAction,execute_cb=self._move_trajectory_cb,auto_start=False)
        self._move_trajectory_as.start()

    def _freedrive_cb(self, msg):
        if msg.data:
            cmd = 'set robotmode freedrive\n'
        else:
            cmd = 'set robotmode run\n'
        self._urscript_pub.publish(String(cmd))

    def _servoing_cb(self, msg):
        self._running_trajectory = False

        cmd = ''
        if msg.motion_type == Servoing.CIRCULAR:
            cmd = self.__servoing_circular(msg.target_pose,msg.radius,msg.velocity,msg.acceleration)
        elif msg.motion_type == Servoing.JOINT:
            if msg.use_ur_ik:
                cmd = self.__servoing_joint_pose(msg.target_pose,msg.velocity,msg.acceleration)
            else:
                cmd = self.__servoing_joint_joints(msg.target_joints.positions,msg.velocity,msg.acceleration)

        self._urscript_pub.publish(String(cmd))

    def __servoing_joint_joints(self, js, velocity, acceleration):
        if self._time_scalars != None:
            j = max([self._time_scalars[i] * abs(js[i] - self._js[i]) for i in range(0,len(self._ordering))])
            if j > 0:
                m = math.log(j)
                if m <= MIN_M:
                    time = MIN_TIME
                    #print 'min', time
                elif m >= MAX_M:
                    time = MAX_TIME
                    #print 'max', time
                else:
                    time = (MAX_TIME - MIN_TIME) / (MAX_M - MIN_M) * (m - MIN_M) + MIN_TIME
                    #print 'linear', time, m
            else:
                #print 'zero-case'
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
        cmd += "\tservoc(rx,ry,rz,px,py,pz,pose_target,{0},{1},{2})\n".format(acceleration,velocity,radius)
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
            if move.motion_type == MoveTrajectoryGoal.CIRCULAR:
                cmd += self.__move_circular(move)
            elif move.motion_type == MoveTrajectoryGoal.JOINT:
                if move.use_ur_ik:
                    cmd += self.__move_joint_pose(move)
                else:
                    cmd += self.__move_joint_joints(move)
            elif move.motion_type == MoveTrajectoryGoal.LINEAR:
                cmd += self.__move_linear(move)
            elif move.motion_type == MoveTrajectoryGoal.PROCESS:
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
        self._urscript.publish(String(cmd))

        feedback = MoveTrajectoryFeedback()
        feedback.message = 'published urscript'
        self._move_trajectory_as.publish_feedback(feedback)

        # wait until movement has ceased
        (pos,rot) = self._tf_listener.lookupTransform(self._ee_frame,self._base_frame,rospy.Time(0))
        prev_pos = [pos]
        prev_quat = [Quaternion(rot[3],rot[0],rot[1],rot[2])]

        preempted = False
        in_steady_state = False
        while not in_steady_state and not preempted:

            # check if forced to stop
            if not self._running_trajectory:
                break

            if self._server.is_preempt_requested():
                preempted = True
                continue

            # waiting timestep
            rospy.sleep(0.1)

            # check steady state on end-effector
            try:
                (pos,rot) = self._tf_listener.lookupTransform(self._ee_frame,self._base_frame,rospy.Time(0))
                quat = Quaternion(rot[3],rot[0],rot[1],rot[2])

                # update pose comparison window
                prev_pos.append(pos)
                prev_quat.append(quat)
                if len(prev_pos) > self._steady_state_length:
                    prev_pos.pop(0)
                    prev_quat.pop(0)

                # compute error and steady_state
                if len(prev_pos) > 1:
                    # compute average error for all poses
                    err_sum = 0
                    differences = len(prev_pos)-1
                    for i in range(0,differences):
                        err_sum += self.__pose_difference()

                    err_ave = err_sum / differences
                    in_steady_state = err_ave < self._steady_state_threshold

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        # Determine end condition
        if not preempted:
            if self._running_trajectory:
                self._running_trajectory = False
                feedback = MoveTrajectoryFeedback()
                feedback.message = 'in steady state'
                self._move_trajectory_as.publish_feedback(feedback)
            else:
                result.message = 'extern behavior prematurely stopped action'
                result.status = False

            self._move_trajectory_as.set_succeeded(result)
        else:
            result.message = 'action execution preempted'
            result.status = False
            self._urscript_pub.publish(String("stopj({0})\n".format(1.2)))
            self._server.set_preempted(result)

    def __pose_difference(self, pos_1, quat_1, pos_2, quat_2):
        pos_dist = math.sqrt(sum([pow(pos_2[i] - pos_1[i],2) for i in range(0,3)]))
        quat_dist = quat_2.absolute_distance(quat_1)
        return pos_dist + quat_dist

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
        js = msg.target_joints.positions

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


if __name__ == "__main__":
    rospy.init_node('ur_controller')

    gain = rospy.get_param('~gain')
    ee_frame = rospy.get_param('~ee_frame')
    lookahead = rospy.get_param('~lookahead')
    base_frame = rospy.get_param('~base_frame')
    time_scalars = rospy.get_param('~time_scalars',None)
    timestep = rospy.get_param('~timestep',MIN_TIME)
    steady_state_length = rospy.get_param('~steady_state_length',DEFAULT_STEADY_STATE_LENGTH)
    steady_state_threshold = rospy.get_param('~steady_state_threshold',DEFAULT_STEADY_STATE_THRESHOLD)

    node = URController(ee_frame,base_frame,gain,lookahead,time_scalars,timestep,
                        steady_state_length,steady_state_threshold)
    rospy.spin()
