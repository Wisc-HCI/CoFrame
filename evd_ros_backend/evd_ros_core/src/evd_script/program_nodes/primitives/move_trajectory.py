'''
Moves a robot according to a preplanned trajectory. This wraps the trajectory
data structure with additional movement parameterization needed to actually
execute on the robot.

TODO implement thing token movement behavior
'''

from ..primitive import Primitive
from ...data.geometry.position import Position
from ...data.geometry.orientation import Orientation


class MoveTrajectory(Primitive):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'move-trajectory' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Primitive.full_type_string() + cls.type_string()

    def __init__(self, trajectory_uuid=None, manual_safety=False, type='', 
                 name='', uuid=None, parent=None, append_type=True, 
                 editable=True, deleteable=True, description=''):

        self._trajectory_uuid = None
        self._manual_safety = None

        super(MoveTrajectory,self).__init__(
            type=MoveTrajectory.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        self.manual_safety = manual_safety
        self.trajectory_uuid = trajectory_uuid

    def to_dct(self):
        tmp = self.context # This guarantees that the patch is applied if possible

        msg = super(MoveTrajectory,self).to_dct()
        msg.update({
            'trajectory_uuid': self.trajectory_uuid,
            'manual_safety': self.manual_safety,
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            type=dct['type'],
            append_type=False,
            editable=dct['editable'],
            deleteable=dct['deleteable'],
            description=dct['description'],
            uuid=dct['uuid'],
            manual_safety=dct['manual_safety'],
            trajectory_uuid=dct['trajectory_uuid'])

    '''
    Data accessor/modifier methods
    '''

    @property
    def manual_safety(self):
        return self._manual_safety

    @manual_safety.setter
    def manual_safety(self, value):
        if self._manual_safety != value:
            self._manual_safety = value
            self.updated_attribute('manual_safety','set')

    @property
    def trajectory_uuid(self):
        return self._trajectory_uuid

    @trajectory_uuid.setter
    def trajectory_uuid(self, value):
        if self._trajectory_uuid != value:
            self._trajectory_uuid = value
            self.updated_attribute('trajectory_uuid','set')

    def set(self, dct):
        if 'start_location_uuid' in dct.keys():
            self.start_location_uuid = dct['start_location_uuid']

        if 'end_location_uuid' in dct.keys():
            self.end_location_uuid = dct['end_location_uuid']

        if 'manual_safety' in dct.keys():
            self.manual_safety = dct['manual_safety']

        if 'trajectory_uuid' in dct.keys():
            self.trajectory_uuid = dct['trajectory_uuid']

        super(MoveTrajectory,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):
        super(MoveTrajectory,self).deep_update()

        self.updated_attribute('manual_safety','update')
        self.updated_attribute('start_location_uuid','update')
        self.updated_attribute('end_location_uuid','update')
        self.updated_attribute('trajectory_uuid','update')

    def shallow_update(self):
        super(MoveTrajectory,self).shallow_update()

        self.updated_attribute('manual_safety','update')
        self.updated_attribute('start_location_uuid','update')
        self.updated_attribute('end_location_uuid','update')
        self.updated_attribute('trajectory_uuid','update')

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        hooks.active_primitive = self

        traj = self.context.get_trajectory(self.trajectory_uuid)
        loc = self.context.get_location(traj.end_location_uuid)
        hooks.tokens['robot']['state']['position'] = loc.position.to_simple_dct()
        hooks.tokens['robot']['state']['orientation'] = loc.orientation.to_simple_dct()
        hooks.tokens['robot']['state']['joints'] = loc.joints

        #TODO handle thing movement

        return self.parent

    def realtime_execution(self, hooks):
        hooks.active_primitive = self
        next = self

        if not self.uuid in hooks.state.keys():
            hooks.robot_interface.is_acked('arm') # clear prev ack
            hooks.state[self.uuid] = 'pending'

            traj = self.context.get_trajectory(self.trajectory_uuid)
            hooks.robot_interface.move_trajectory_async(traj, self.manual_safety)

        else:
            resp = hooks.robot_interface.is_acked('arm')
            if resp != None:
                if resp:
                    del hooks.state[self.uuid]
                    next = self.parent

                else:
                    raise Exception('Robot NACKed')

        status = hooks.robot_interface.get_status()
        hooks.tokens['robot']['state']['position'] = Position.from_ros(status.arm_pose.position).to_simple_dct()
        hooks.tokens['robot']['state']['orientation'] = Orientation.from_ros(status.arm_pose.orientation).to_simple_dct()
        hooks.tokens['robot']['state']['joints'] = status.arm_joints

        #TODO handle thing movement

        return next