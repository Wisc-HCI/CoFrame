'''
Moves a robot according to a preplanned trajectory. This wraps the trajectory
data structure with additional movement parameterization needed to actually
execute on the robot.

TODO implement thing token movement behavior
'''

import numpy as np

from ..primitive import Primitive
from ... import BOOLEAN_TYPE
from ...data_nodes.trajectory import Trajectory
from ...data_nodes.geometry.pose import Pose
from ...data_nodes.geometry.position import Position
from ...data_nodes.geometry.orientation import Orientation


class MoveTrajectory(Primitive):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Move Trajectory'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'move-trajectory' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Primitive.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Primitive.template()
        template['parameters'].append({
            'type': BOOLEAN_TYPE,
            'key': 'manual_safety',
            'is_uuid': False,
            'is_list': False
        })
        template['parameters'].append({
            'type': Trajectory.full_type_string(),
            'key': 'trajectory_uuid',
            'is_uuid': True,
            'is_list': False
        })
        return template

    def __init__(self, trajectory_uuid=None, manual_safety=False, parameters=None, type='', 
                 name='', uuid=None, parent=None, append_type=True, 
                 editable=True, deleteable=True, description=''):

        if parameters == None:
            parameters = {
                'manual_safety': manual_safety,
                'trajectory_uuid': trajectory_uuid
            }

        super(MoveTrajectory,self).__init__(
            type=MoveTrajectory.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description,
            parameters=parameters)

    '''
    Data accessor/modifier methods
    '''

    @property
    def manual_safety(self):
        return self._parameters['manual_safety']

    @manual_safety.setter
    def manual_safety(self, value):
        if self._parameters['manual_safety'] != value:
            self._parameters['manual_safety'] = value
            self.updated_attribute('parameters.manual_safety','set')

    @property
    def trajectory_uuid(self):
        return self._parameters['trajectory_uuid']

    @trajectory_uuid.setter
    def trajectory_uuid(self, value):
        if self._parameters['trajectory_uuid'] != value:
            self._parameters['trajectory_uuid'] = value
            self.updated_attribute('parameters.trajectory_uuid','set')

    def set(self, dct):

        if 'manual_safety' in dct.keys():
            self.manual_safety = dct['manual_safety']

        if 'trajectory_uuid' in dct.keys():
            self.trajectory_uuid = dct['trajectory_uuid']

        super(MoveTrajectory,self).set(dct)

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        hooks.active_primitive = self

        # Compute transform from thing to robot
        Ttr = None
        thing_uuid = hooks.tokens['robot']['state']['gripper']['grasped_thing']
        if None != thing_uuid:
            thing = self.context.get_thing(thing_uuid)
            old_thing_pose = thing.to_simple_dct()
            old_robot_pose = {
                'position': dict(hooks.tokens['robot']['state']['position']),
                'orientation': dict(hooks.tokens['robot']['state']['orientation'])
            }
            Ttr, _ = Pose.compute_relative(Pose.from_simple_dct(old_thing_pose),Pose.from_simple_dct(old_robot_pose))

        # update robot state
        traj = self.context.get_trajectory(self.trajectory_uuid)
        loc = self.context.get_location(traj.end_location_uuid)
        hooks.tokens['robot']['state']['position'] = loc.position.to_simple_dct()
        hooks.tokens['robot']['state']['orientation'] = loc.orientation.to_simple_dct()
        hooks.tokens['robot']['state']['joints'] = loc.joints

        # Compute transform thing to world
        if None != thing_uuid:
            Trw = Pose.from_simple_dct({
                'position': dict(hooks.tokens['robot']['state']['position']),
                'orientation': dict(hooks.tokens['robot']['state']['orientation'])
            }).to_matrix()
            Ttw = Pose.matrix_inverse(np.matmul(Ttr,Pose.matrix_inverse(Trw)))
            new_thing_pose = Pose.from_matrix(Ttw).to_simple_dct()
            hooks.tokens[thing_uuid]['state'].update(new_thing_pose)

        return self.parent

    def realtime_execution(self, hooks):
        hooks.active_primitive = self
        next = self

        thing_uuid = hooks.tokens['robot']['state']['gripper']['grasped_thing']

        if not self.uuid in hooks.state.keys():

            # Compute transform from thing to robot
            Ttr = None
            if None != thing_uuid:
                Ttr, _ = Pose.compute_relative(
                    Pose.from_simple_dct(hooks.tokens[thing_uuid]['state']),
                    Pose.from_simple_dct(hooks.tokens['robot']['state']))
                
            # Set initial state and start action
            hooks.robot_interface.is_acked('arm') # clear prev ack
            hooks.state[self.uuid] = 'pending'
            traj = self.context.get_trajectory(self.trajectory_uuid)
            hooks.robot_interface.move_trajectory_async(traj, self.manual_safety)

        else:
            resp = hooks.robot_interface.is_acked('arm')
            if resp != None:
                if resp:
                    next = self.parent

                else:
                    raise Exception('Robot NACKed')

        status = hooks.robot_interface.get_status()
        hooks.tokens['robot']['state']['position'] = Position.from_ros(status.arm_pose.position).to_simple_dct()
        hooks.tokens['robot']['state']['orientation'] = Orientation.from_ros(status.arm_pose.orientation).to_simple_dct()
        hooks.tokens['robot']['state']['joints'] = status.arm_joints

        #Set thing state / Compute transform thing to world
        if None != thing_uuid:
            Trw = Pose.from_simple_dct(hooks.tokens['robot']['state']).to_matrix()
            Ttr = hooks.state[self.uuid]['Ttr']
            Ttw = Pose.matrix_inverse(np.matmul(Ttr,Pose.matrix_inverse(Trw)))
            new_thing_pose = Pose.from_matrix(Ttw).to_simple_dct()
            hooks.tokens[thing_uuid]['state'].update(new_thing_pose)

        if next == self.parent:
            del hooks.state[self.uuid]
        return next