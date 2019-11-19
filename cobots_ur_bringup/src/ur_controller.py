#!/usr/bin/env python

import tf
import time
import math
import rospy
import actionlib
import roslibpy
import roslibpy.actionlib

from std_msgs.msg import Bool, String
from ur_msgs.msg import RobotModeDataMsg
from cobots_core.msg import Stop, Servo, Move
from geometry_msgs.msg import Pose, Quaternion, Vector3
from cobots_core.msg import MoveTrajectoryAction, MoveTrajectoryGoal, MoveTrajectoryResult, MoveTrajectoryFeedback


MIN_M = -30.0
MAX_M = 1.0
MIN_TIME = 0.05
MAX_TIME = 0.9


class ActionServerROStoBridgeTranslation:

    def __init__(self, bridge):
        self._bridge = bridge

    def is_preempt_requested(self):
        return self._bridge.is_preempt_requested()

    def publish_feedback(self, msg):
        return self._bridge.publish_feedback({
            'message': msg.message
        })

    def set_succeeded(self, msg):
        return self._bridge.set_succeeded({
            'status': msg.status,
            'message': msg.message
        })

    def set_preempted(self, msg):
        return self._bridge.set_preempted()


class URController:

    def __init__(self, mode, gain, lookahead, time_scalars, timestep,
                 rosbridge_host=None, rosbridge_port=None, bridge_name_prefix=None):
        self._mode = mode
        self._gain = gain
        self._timestep = timestep
        self._lookahead = lookahead
        self._running_trajectory = False
        self._time_scalars = time_scalars
        self._robot_state = RobotModeDataMsg()

        self._urscript_pub = rospy.Publisher('ur_driver/URScript',String,queue_size=10)

        if mode == 'ros':
            self._freedrive_sub = rospy.Subscriber('robot_control/freedrive',Bool,self._freedrive_cb)
            self._servoing_sub = rospy.Subscriber('robot_control/servoing',Servo,self._servoing_cb)
            self._stop_sub = rospy.Subscriber('robot_control/stop',Stop,self._stop_cb)

            self._move_trajectory_as = actionlib.SimpleActionServer('robot_control/move_trajectory',MoveTrajectoryAction,execute_cb=self._move_trajectory_cb,auto_start=False)
            self._move_trajectory_as.start()

        elif mode == 'bridge':
            self._bridge_client = roslibpy.Ros(host=rosbridge_host,port=rosbridge_port)

            not_setup = True
            while not rospy.is_shutdown() and not_setup:
                try:
                    self._bridge_client.run()
                    not_setup = False
                except:
                    print 'Waiting for ROSBridge to connect'
                rospy.sleep(0.25)
            print 'ROSBridge connected'

            self._ur_mode_pub = roslibpy.Topic(self._bridge_client, '{}/ur_driver/robot_mode_state'.format(bridge_name_prefix),'ur_msgs/RobotModeDataMsg')
            rospy.Timer(rospy.Duration(0.1), self._ur_mode_republish_cb)

            self._freedrive_sub = roslibpy.Topic(self._bridge_client, '{}/robot_control/freedrive'.format(bridge_name_prefix), 'std_msgs/Bool')
            self._freedrive_sub.subscribe(self._freedrive_bridge_cb)

            self._servoing_sub = roslibpy.Topic(self._bridge_client, '{}/robot_control/servoing'.format(bridge_name_prefix), 'cobots_core/Servo')
            self._servoing_sub.subscribe(self._servoing_bridge_cb)

            self._stop_sub = roslibpy.Topic(self._bridge_client, '{}/robot_control/stop'.format(bridge_name_prefix), 'cobots_core/Stop')
            self._stop_sub.subscribe(self._stop_bridge_cb)

            self._move_trajectory_bridge_as = roslibpy.actionlib.SimpleActionServer(self._bridge_client, '{}/robot_control/move_trajectory'.format(bridge_name_prefix), 'cobots_core/MoveTrajectoryAction')
            self._move_trajectory_bridge_as.start(self._move_trajectory_bridge_cb)
            self._move_trajectory_as = ActionServerROStoBridgeTranslation(self._bridge_client)

        else:
            raise Exception('Invalid ROS interface mode selected: {}'.format(mode))

        self._ur_mode_sub = rospy.Subscriber('ur_driver/robot_mode_state',RobotModeDataMsg,self._ur_mode_cb)

    def _ur_mode_cb(self, msg):
        self._robot_state = msg

    def _ur_mode_republish_cb(self, event):
        self._ur_mode_pub.publish({
            'timestamp': self._robot_state.timestamp,
            'is_robot_connected': self._robot_state.is_robot_connected,
            'is_real_robot_enabled': self._robot_state.is_real_robot_enabled,
            'is_power_on_robot': self._robot_state.is_power_on_robot,
            'is_emergency_stopped': self._robot_state.is_emergency_stopped,
            'is_protective_stopped': self._robot_state.is_protective_stopped,
            'is_program_running': self._robot_state.is_program_running,
            'is_program_paused': self._robot_state.is_program_paused
        })

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
        print cmd
        self._urscript_pub.publish(String(cmd))

    def _freedrive_bridge_cb(self, msg):
        self._freedrive_cb(Bool(msg['data']))

    def _servoing_cb(self, msg):
        self._running_trajectory = False

        cmd = ''
        if msg.motion_type == Servo.CIRCULAR:
            cmd = self.__servoing_circular(msg.target_pose,msg.radius,msg.velocity,msg.acceleration)
        elif msg.motion_type == Servo.JOINT:
            if msg.use_ur_ik:
                cmd = self.__servoing_joint_pose(msg.target_pose,msg.velocity,msg.acceleration)
            else:
                cmd = self.__servoing_joint_joints(msg.target_joints.positions,msg.velocity,msg.acceleration)

        print cmd

        self._urscript_pub.publish(String(cmd))

    def _servoing_bridge_cb(self, msg):
        self._servoing_cb(Servo(
            motion_type=msg['motion_type'],
            use_ur_ik=msg['use_ur_ik'],
            target_pose=self.__translate_pose(msg['target_pose']),
            target_joints=self.__translate_joint_trajectory_point(msg['target_joints']),
            radius=msg['radius'],
            acceleration=msg['acceleration'],
            velocity=msg['velocity']))

    def __translate_pose(self, dct):
        return Pose(
            position=Vector3(
                x=dct['position']['x'],
                y=dct['position']['y'],
                z=dct['position']['z']),
            orientation=Quaternion(
                x=dct['orientation']['x'],
                y=dct['orientation']['y'],
                z=dct['orientation']['z'],
                w=dct['orientation']['w']))

    def __translate_joint_trajectory_point(self, dct):
        return JointTrajectoryPoint(
            positions=dct['positions'],
            velocities=dct['velocities'],
            accelerations=dct['accelerations'],
            effort=dct['effort'],
            time_from_start=dct['time_from_start'])

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

        print cmd

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

    def _move_trajectory_bridge_cb(self, goal):
        moves = []
        for md in goal['moves']:
            moves.append(Move(
                motion_type=md['motion_type'],
                use_ur_ik=md['use_ur_ik'],
                target_pose=self.__translate_pose(md['target_pose']),
                target_joints=self.__translate_joint_trajectory_point(md['target_joints']),
                path_pose=self.__translate_pose(md['path_pose']),
                radius=md['radius'],
                acceleration=md['acceleration'],
                velocity=md['velocity'],
                time=md['time'],
                circular_constrained=md['circular_constrained']))
        self._move_trajectory_cb(MoveTrajectoryGoal(moves=moves))

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

        print cmd

        self._urscript_pub.publish(String(cmd))
        self._running_trajectory = False

    def _stop_bridge_cb(self, msg):
        self._stop_cb(Stop(
            motion_type=msg['motion_type'],
            acceleration=msg['acceleration']))


if __name__ == "__main__":
    rospy.init_node('ur_controller')

    mode = rospy.get_param('~mode') #ros, bridge
    gain = rospy.get_param('~gain')
    lookahead = rospy.get_param('~lookahead')
    time_scalars = rospy.get_param('~time_scalars',None)
    timestep = rospy.get_param('~timestep',MIN_TIME)

    rosbridge_host = rospy.get_param('~rosbridge_host',None)
    rosbridge_port = rospy.get_param('~rosbridge_port',None)
    bridge_name_prefix = rospy.get_param('~bridge_name_prefix',None)

    node = URController(mode,gain,lookahead,time_scalars,timestep,
                        rosbridge_host, rosbridge_port, bridge_name_prefix)
    rospy.spin()
