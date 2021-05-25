'''
Moves a robot according to a preplanned trajectory. This wraps the trajectory
data structure with additional movement parameterization needed to actually
execute on the robot.

TODO implement real-time move trajectory behavior
TODO implement thing token movement behavior
'''

from ..primitive import Primitive
from ...data.trajectory import Trajectory
from ...node_parser import NodeParser

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

    def __init__(self, startLocUuid=None, endLocUuid=None, trajectory=None, trajectory_uuid=None,
                 type='', name='', uuid=None, parent=None, append_type=True):

        self._context_patch = None
        self._start_location_uuid = None
        self._end_location_uuid = None
        self._trajectory_uuid = None

        super(MoveTrajectory,self).__init__(
            type=MoveTrajectory.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.start_location_uuid = startLocUuid
        self.end_location_uuid = endLocUuid

        if trajectory != None and trajectory_uuid != None:
            raise Exception('Cannot supply both a default trajectory and a trajectory id already in context')
        elif trajectory != None:
            self.trajectory = trajectory
        elif trajectory_uuid != None:
            self.trajectory_uuid = trajectory_uuid
        else:
            self.trajectory = Trajectory(self.start_location_uuid,self.end_location_uuid)

    def to_dct(self):
        tmp = self.context # This guarantees that the patch is applied if possible

        msg = super(MoveTrajectory,self).to_dct()
        msg.update({
            'start_location_uuid': self.start_location_uuid,
            'end_location_uuid': self.end_location_uuid,
        })

        if self._context_patch == None:
            msg['trajectory_uuid'] = self.trajectory_uuid
        else:
            msg['trajectory'] = self.trajectory.to_dct()

        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            type=dct['type'],
            append_type=False,
            uuid=dct['uuid'],
            startLocUuid=dct['start_location_uuid'],
            endLocUuid=dct['end_location_uuid'],
            trajectory=NodeParser(dct['trajectory'], enforce_type=Trajectory.type_string(trailing_delim=False)) if 'trajectory' in dct.keys() else None,
            trajectory_uuid=dct['trajectory_uuid'] if 'trajectory_uuid' in dct.keys() else None)

    def on_delete(self):
        self.context.delete_trajectory(self.trajectory_uuid)

        super(MoveTrajectory,self).on_delete()

    '''
    Data accessor/modifier methods
    '''

    @property
    def context(self):
        # Need to patch context when none exists
        realContext = super(MoveTrajectory,self).context
        if realContext == None:
            if self._context_patch == None:
                self._context_patch = ContextPatch(self)
            return self._context_patch
        else:
            if self._context_patch != None:
                self._context_patch.update_context(realContext)
                self._context_patch = None
            return realContext

    @property
    def start_location_uuid(self):
        return self._start_location_uuid

    @start_location_uuid.setter
    def start_location_uuid(self, value):
        if self._start_location_uuid != value:
            self._start_location_uuid = value

            t = self.context.get_trajectory(self.trajectory_uuid)
            if t != None:
                t.start_location_uuid = self._start_location_uuid

            self.updated_attribute('start_location_uuid','set')

    @property
    def end_location_uuid(self):
        return self._end_location_uuid

    @end_location_uuid.setter
    def end_location_uuid(self, value):
        if self._end_location_uuid != value:
            self._end_location_uuid = value

            t = self.context.get_trajectory(self.trajectory_uuid)
            if t != None:
                t.end_location_uuid = self._end_location_uuid

            self.updated_attribute('end_location_uuid','set')

    @property
    def trajectory(self):
        return self.context.get_trajectory(self.trajectory_uuid)

    @trajectory.setter
    def trajectory(self, value):
        if self._trajectory_uuid != value.uuid:
            self.context.delete_trajectory(self.trajectory_uuid)
            self.context.add_trajectory(value)
            self.trajectory_uuid = value.uuid

            self.updated_attribute('trajectory','set')

    @property
    def trajectory_uuid(self):
        return self._trajectory_uuid

    @trajectory_uuid.setter
    def trajectory_uuid(self, value):
        if self._trajectory_uuid != value:
            if value == None:
                raise Exception('Id must be defined')

            traj = self.context.get_trajectory(value)
            if traj == None and self._context_patch == None:
                raise Exception('Id must have a trajectory already in context')

            elif traj == None and self._context_patch != None:
                self._context_patch.add_pending_trajectory(
                    uuid=value,
                    startLoc=self.start_location_uuid,
                    endLoc=self.end_location_uuid)

            elif traj != None:
                if traj.start_location_uuid != self.start_location_uuid:
                    raise Exception('Trajectory must have matching start location to primitive if set by id')

                if traj.end_location_uuid != self.end_location_uuid:
                    raise Exception('Trajectory must have matching end location to primitive if set by id')

            self._trajectory_uuid = value
            self.updated_attribute('trajectory_uuid','set')

    def set(self, dct):
        if 'start_location_uuid' in dct.keys():
            self.start_location_uuid = dct['start_location_uuid']

        if 'end_location_uuid' in dct.keys():
            self.end_location_uuid = dct['end_location_uuid']

        if 'trajectory' in dct.keys():
            self.trajectory = NodeParser(dct['trajectory'], enforce_type=Trajectory.type_string(trailing_delim=False))

        if 'trajectory_uuid' in dct.keys():
            self.trajectory_uuid = dct['trajectory_uuid']

        super(MoveTrajectory,self).set(dct)

    '''
    Update Methods
    '''

    def late_construct_update(self):
        tmp = self.context # This guarantees that the patch is applied if possible

        super(MoveTrajectory,self).late_construct_update()

    def deep_update(self):
        tmp = self.context # This guarantees that the patch is applied if possible

        super(MoveTrajectory,self).deep_update()

        self.updated_attribute('start_location_uuid','update')
        self.updated_attribute('end_location_uuid','update')
        self.updated_attribute('trajectory_uuid','update')
        self.updated_attribute('trajectory','update')

    def shallow_update(self):
        super(MoveTrajectory,self).shallow_update()

        self.updated_attribute('start_location_uuid','update')
        self.updated_attribute('end_location_uuid','update')
        self.updated_attribute('trajectory_uuid','update')
        self.updated_attribute('trajectory','update')

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        hooks.active_primitive = self

        loc = self.context.get_location(self.end_location_uuid)
        hooks.tokens['robot']['state']['position'] = loc.position.to_simple_dct()
        hooks.tokens['robot']['state']['orientation'] = loc.orientation.to_simple_dct()
        hooks.tokens['robot']['state']['joints'] = loc.joints

        return self.parent

    def realtime_execution(self, hooks):

        # TODO actually execute move trajectory behavior

        return self.symbolic_execution(hooks)


class ContextPatch(object):

    def __init__(self, parent):
        self._trajectories = {}
        self._parent = parent
        self._pending = {}

    @property
    def parent(self):
        return self._parent

    @property
    def trajectories(self):
        traj = {uuid: self._trajectories[uuid] for uuid in self._trajectories.keys()} # copy
        traj.update({uuid: 'pending' for uuid in self._pending.keys()})
        return traj

    @trajectories.setter
    def trajectories(self, value):
        for t in self._trajectories:
            t.remove_from_cache()
        self._trajectories = {}

        for t in value:
            self._trajectories[t.uuid] = t
            t.parent = self

    def get_trajectory(self, uuid):
        if uuid in self._trajectories.keys():
            return self._trajectories[uuid]
        elif uuid in self._pending.keys():
            return 'pending'
        else:
            return None

    def add_trajectory(self, trajectory):
        trajectory.parent = self
        self._trajectories[trajectory.uuid] = trajectory

    def delete_trajectory(self, uuid):
        if uuid in self._trajectories.keys():
            self._trajectories.pop(uuid).remove_from_cache()
        elif uuid in self._pending.keys():
            self._pending.pop(uuid)

    def update_context(self, context):

        for traj in self._trajectories.values():
            context.add_trajectory(traj)

        for uuid in self._pending.keys():
            traj = context.get_trajectory(uuid)
            if traj == None:
                raise Exception('Pending trajectory was not found when context was assigned')

            if traj.start_location_uuid != self._pending[uuid]['startLoc']:
                raise Exception('Pending trajectory start location does not match one in context')

            if traj.end_location_uuid != self._pending[uuid]['endLoc']:
                raise Exception('Pending trajectory end location does not match one in context')

        self._trajectories = {}
        self._pending = {}

    def child_changed_event(self, attribute_trace):
        self.parent.child_changed_event(attribute_trace)

    def add_pending_trajectory(self, uuid, startLoc, endLoc):
        self._pending[uuid] = {'startLoc': startLoc, 'endLoc': endLoc}
