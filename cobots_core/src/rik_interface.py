'''
'''

import time
import math
import rospy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, Quaternion, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from RelaxedIK.relaxedIK import get_relaxedIK_from_info_file
from RelaxedIK.Utils.yaml_utils import get_relaxedIK_yaml_obj_from_info_file_name


DEFAULT_STEP_SIZE = 0.005
DEFAULT_HOLD_COUNT = 5
DEFAULT_STEADY_STATE_TIMEOUT = 0.5
DEFAULT_STEADY_STATE_BUFFER_LENGTH = 3
DEFAULT_STEADY_STATE_FINAL_THRESHOLD = 0.001
DEFAULT_STEADY_STATE_KEYPOINT_THRESHOLD = 0.5


class RelaxedIKInterface:

    def __init__(self, path_to_rik_src, info_file_name,
                 step_size=DEFAULT_STEP_SIZE,
                 hold_count=DEFAULT_HOLD_COUNT,
                 joint_diff_weights=None,
                 steady_state_timeout=DEFAULT_STEADY_STATE_TIMEOUT,
                 steady_state_buffer_length=DEFAULT_STEADY_STATE_BUFFER_LENGTH,
                 steady_state_final_threshold=DEFAULT_STEADY_STATE_FINAL_THRESHOLD,
                 steady_state_keypoint_threshold=DEFAULT_STEADY_STATE_KEYPOINT_THRESHOLD):

        self._step_size = step_size
        self._hold_count = hold_count
        self._joint_diff_weights = joint_diff_weights

        self._steady_state_timeout = steady_state_timeout
        self._steady_state_buffer_length = steady_state_buffer_length
        self._steady_state_final_threshold = steady_state_final_threshold
        self._steady_state_keypoint_threshold = steady_state_keypoint_threshold

        self._relaxedIK = get_relaxedIK_from_info_file(path_to_rik_src,info_file_name)
        self._relaxedIK.vars.rotation_mode = 'absolute'
        self._relaxedIK.vars.position_mode = 'absolute'
        if self._relaxedIK.vars.robot.numChains != 1:
            raise ValueError('Only one chain supported in this behavior planner')

    def initialize_joint_state(self, initial_js, initial_pose):

        # set joint_state in RelaxedIK
        names = self._relaxedIK.vars.joint_order
        start_joints = []
        for i in range(0,len(names)):
            n = names[i]
            k = initial_js.name.index(n)
            start_joints.append(initial_js.position[k])
        self._relaxedIK.vars.relaxedIK_vars_update(start_joints)
        self._relaxedIK.vars.xopt = start_joints

        # run until steady state for initial pose
        _j = self._rik_path_loop([initial_pose])

    def plan_path(self, initial_pose, target_pose, initial_js=None):

        # initialize joint state for relaxed-ik for replan
        if initial_js != None:
            self.initialize_joint_state(initial_js,initial_pose)

        # run until steady state for target pose
        linear_path = self._generate_linear_path(initial_pose,target_pose)
        joints = self._rik_path_loop(linear_path)

        # package joint trajectory
        return self._package_trajectory(joints)

    def real_time_servoing(self, target_pose):
        pos = [ target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z ]
        rot = [ target_pose.orientation.w,
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z ]
        xopt = self._relaxedIK.solve([pos],[rot])

        point = JointTrajectoryPoint(positions=xopt,
                                     velocities=[0]*len(xopt),
                                     accelerations=[0]*len(xopt),
                                     effort=[0]*len(xopt))
        return point

    def _generate_linear_path(self, current, target):

        # define position step vector
        dif = [target.position.x-current.position.x,
               target.position.y-current.position.y,
               target.position.z-current.position.z]
        lengthDifVector = math.sqrt(sum([dif[i]**2 for i in range(0,3)]))
        if lengthDifVector <= 0:
            return [target]
        stepVector = [self._step_size / lengthDifVector * dif[i] for i in range(0,3)]

        # generate position array
        positions = []
        lengthNext = 0
        lengthPath = math.sqrt(sum([dif[i]**2 for i in range(0,3)]))
        prevBuf = [current.position.x,current.position.y,current.position.z]
        while True:
            nextBuf = [prevBuf[i] + stepVector[i] for i in range(0,3)]
            lengthNext += self._step_size

            if lengthNext >= lengthPath:
                positions.append(target.position)
                break
            else:
                positions.append(Vector3(nextBuf))
            prevBuf = nextBuf

        # generate orientation array
        steps = len(positions)
        orientations = []
        if steps == 1:
            orientations.append([target.orientation.x,target.orientation.y,target.orientation.z,target.orientation.w])
        elif steps:
            for i in range(0,steps):
                orientations.append(tf.transformations.quaternion_slerp(
                    [current.orientation.x,current.orientation.y,current.orientation.z,current.orientation.w],
                    [target.orientation.x,target.orientation.y,target.orientation.z,target.orientation.w],
                    i/(steps-1.0)))

        # format pose array
        path = [self._format_pose(positions[i],orientations[i]) for i in range(0,len(positions))]

        # append hold poses
        for h in range(0,self._hold_count):
            path.append(target)

        return path

    def _format_pose(self, position, orientation):
        return Pose(
            position=Vector3(
                x=position[0],
                y=position[1],
                z=position[2]),
            orientation=Quaternion(
                x=orientation[0],
                y=orientation[1],
                z=orientation[2],
                w=orientation[3]))

    def _rik_path_loop(self, path):
        joints = [self._relaxedIK.vars.xopt]
        prev_buffer = []
        pending = True

        for i in range(0,len(path)):
            pose = path[i]

            # while moving to pose
            base_time = time.time()
            while pending and (time.time() - base_time) <= self._steady_state_timeout:

                # get joint step
                xopt = self.real_time_servoing(pose)

                # update joints
                joints.append(xopt)
                prev_buffer.append(xopt)
                if len(prev_buffer) > self._steady_state_buffer_length:
                    prev_buffer.pop(0)

                # determine if in steady state (reached optimal)
                threshold = self._steady_state_final_threshold if i == (len(path) - 1) else self._steady_state_keypoint_threshold
                pending = not self._joint_steady_state_error(prev_buffer,threshold)

            # reset for next iteration
            pending = True

        return joints

    def _joint_difference(self, ja, jb, w=None):
        if w == None:
            w = [1]*len(ja)
        return math.sqrt(sum([w[j] * pow((jb[j] - ja[j]),2) for j in range(0,len(ja))]))

    def _joint_steady_state_error(self, joint_buffer, threshold):
        if len(joint_buffer) > 1:
            # compute average error between joint arrays
            err_sum = 0
            differences = (len(joint_buffer) - 1)
            for i in range(0,differences):
                err_sum += self._joint_difference(joint_buffer[i],joint_buffer[i+1],self._joint_diff_weights)

            err_ave = err_sum / differences
            print err_ave
            return err_ave < threshold
        else:
            return False

    def _package_trajectory(self, joints):
        points = []
        for i in range(0,len(joints)):
            p = JointTrajectoryPoint(positions=joints[i],
                                     velocities=[0]*len(joints[i]),
                                     accelerations=[0]*len(joints[i]),
                                     effort=[0]*len(joints[i]))
            points.append(p)

        trajMsg = JointTrajectory()
        trajMsg.joint_names = self._relaxedIK.vars.joint_order
        trajMsg.points = points
        return trajMsg
