from ..node import Node
from ..data.trajectory import Trajectory


class Primitive(Node):

    '''
    Data structure methods
    '''

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True):
        super(Primitive,self).__init__(
            type='primitive.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)


class MoveTrajectory(Primitive):

    '''
    Data structure methods
    '''

    def __init__(self, startLocUuid=None, endLocUuid=None, trajectories=[],
                 runnableTrajUuid=None, type='', name='', uuid=None, parent=None,
                 append_type=True, create_default=True):

        self._start_location_uuid = None
        self._end_location_uuid = None
        self._trajectories = []
        self._runnable_trajectory_uuid = None

        super(MoveTrajectory,self).__init__(
            type='move-trajectory.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.start_location_uuid = startLocUuid
        self.end_location_uuid = endLocUuid
        self.trajectories = trajectories
        self.runnable_trajectory_uuid = runnableTrajUuid

        if len(self.trajectories) == 0 and create_default:
            self.add_trajectory(Trajectory(self.start_location_uuid,self._end_location_uuid))

    def to_dct(self):
        msg = super(MoveTrajectory,self).to_dct()
        msg.update({
            'start_location_uuid': self.start_location_uuid,
            'end_location_uuid': self.end_location_uuid,
            'trajectories': [t.to_dct() for t in self.trajectories],
            'runnable_trajectory_uuid': self.runnable_trajectory_uuid
        })
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
            trajectories=[Trajectory.from_dct(t) for t in dct['trajectories']],
            runnableTrajUuid=dct['runnable_trajectory_uuid'])

    '''
    Data accessor/modifier methods
    '''

    @property
    def start_location_uuid(self):
        return self._start_location_uuid

    @start_location_uuid.setter
    def start_location_uuid(self, value):
        if self._start_location_uuid != value:
            self._start_location_uuid = value

            for t in self.trajectories:
                t.start_location_uuid = self._start_location_uuid

            self.updated_attribute('start_location_uuid','set')

    @property
    def end_location_uuid(self):
        return self._end_location_uuid

    @end_location_uuid.setter
    def end_location_uuid(self, value):
        if self._end_location_uuid != value:
            self._end_location_uuid = value

            for t in self.trajectories:
                t.end_location_uuid = self._end_location_uuid

            self.updated_attribute('end_location_uuid','set')

    @property
    def trajectories(self):
        return self._trajectories

    @trajectories.setter
    def trajectories(self, value):
        if self._trajectories != value:
            for t in self._trajectories:
                t.remove_from_cache()

            self._trajectories = value
            for t in self._trajectories:
                t.parent = self

            runnableFound = False
            for t in self._trajectories:
                t.parent = self
                if t.uuid == self.runnable_trajectory_uuid:
                    runnableFound = True

            if not runnableFound:
                # must assign a runnable id if more than zero trajectories
                if len(self._trajectories) > 0:
                    self.runnable_trajectory_uuid = self._trajectories[0].uuid
                else:
                    self.runnable_trajectory_uuid = None

            self.updated_attribute('trajectories','set')

    @property
    def runnable_trajectory_uuid(self):
        return self._runnable_trajectory_uuid

    @runnable_trajectory_uuid.setter
    def runnable_trajectory_uuid(self, value):
        if self._runnable_trajectory_uuid != value:
            runUuidFound = False
            for t in self._trajectories:
                if t.uuid == value:
                    runUuidFound = True
                    break

            if not runUuidFound and value != None:
                raise Exception("runnable trajectory uuid not found in trajectories")

            # must always have a runnable uuid set if there is at least one trajectory
            if value == None and len(self.trajectories) > 0:
                self._runnable_trajectory_uuid = self.trajectories[0].uuid
            else:
                self._runnable_trajectory_uuid = value

            self.updated_attribute('runnable_trajectory_uuid','set')

    def add_trajectory(self, t):
        t.parent = self
        self._trajectories.append(t)

        if len(self._trajectories) == 1:
            self.runnable_trajectory_uuid = t.uuid

        self.updated_attribute('trajectories','add')

    def get_trajectory(self, uuid):
        for t in self._trajectories:
            if t.uuid == uuid:
                return t
        return None

    def reorder_trajectories(self, uuid, shift):
        idx = None
        for i in range(0,len(self._trajectories)):
            if self._trajectories[i].uuid == uuid:
                idx = i
                break

        if idx != None:
            shiftedIdx = idx + shift
            if shiftedIdx < 0 or shiftedIdx >= len(self._trajectories):
                raise Exception("Index out of bounds")

            copy = self._trajectories.pop(idx)
            self._trajectories.insert(shiftedIdx,copy) #TODO check to make sure not off by one

            self.updated_attribute('trajectories','reorder')

    def delete_trajectory(self, uuid):
        delIdx = None
        for i in range(0,len(self._trajectories)):
            if self._trajectories[i].uuid == uuid:
                delIdx = i
                break

        if delIdx != None:
            self._trajectories.pop(delIdx).remove_from_cache()

            if uuid == self.runnable_trajectory_uuid:
                if len(self.trajectories) > 0:
                    self.runnable_trajectory_uuid = self.trajectories[0].uuid
                else:
                    self.runnable_trajectory_uuid = None

            self.updated_attribute('trajectories','delete')

    def set(self, dct):
        if 'start_location_uuid' in dct.keys():
            self.start_location_uuid = dct['start_location_uuid']

        if 'end_location_uuid' in dct.keys():
            self.end_location_uuid = dct['end_location_uuid']

        if 'trajectories' in dct.keys():
            self.trajectories = [Trajectory.from_dct(t) for t in dct['trajectories']]

        if 'runnable_trajectory_uuid' in dct.keys():
            self.runnable_trajectory_uuid = dct['runnable_trajectory_uuid']

        super(MoveTrajectory,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):
        for t in self._trajectories:
            t.remove_from_cache()

        super(MoveTrajectory,self).remove_from_cache()

    def add_to_cache(self):
        for t in self._trajectories:
            t.add_to_cache()

        super(MoveTrajectory,self).add_to_cache()

    '''
    Children methods (optional)
    '''

    def delete_child(self, uuid):
        if uuid in [t.uuid for t in self.trajectories]:
            self.delete_trajectory(uuid)


class MoveUnplanned(Primitive):

    '''
    Data structure methods
    '''

    def __init__(self, locUuid, manual_safety=True, type='', name='', uuid=None,
                 parent=None, append_type=True):

        self._location_uuid = None
        self._manual_safety = None

        super(MoveUnplanned,self).__init__(
            type='move-unplanned.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.manual_safety = manual_safety
        self.location_uuid = locUuid

    def to_dct(self):
        msg = super(MoveUnplanned,self).to_dct()
        msg.update({
            'location_uuid': self.location_uuid,
            'manual_safety': self.manual_safety
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            type=dct['type'],
            append_type=False,
            uuid=dct['uuid'],
            locUuid=dct['location_uuid'],
            manual_safety=dct['manual_safety'])

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
    def location_uuid(self):
        return self._location_uuid

    @location_uuid.setter
    def location_uuid(self, value):
        if self._location_uuid != value:
            self._location_uuid = value
            self.updated_attribute('location_uuid','set')

    def set(self, dct):
        if 'location_uuid' in dct.keys():
            self.location_uuid = dct['location_uuid']

        if 'manual_safety' in dct.keys():
            self.manual_safety = dct['manual_safety']

        super(MoveUnplanned,self).set(dct)


class Delay(Primitive):

    '''
    Data structure methods
    '''
    def __init__(self, duration=0, type='', name='', uuid=None, parent=None, append_type=True):
        self._duration = None

        super(Delay,self).__init__(
            type='delay.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.duration = duration

    def to_dct(self):
        msg = super(Delay,self).to_dct()
        msg.update({
            'duration': self.duration,
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            type=dct['type'],
            append_type=False,
            uuid=dct['uuid'],
            duration=dct['duration'])

    '''
    Data accessor/modifier methods
    '''

    @property
    def duration(self):
        return self._duration

    @duration.setter
    def duration(self, value):
        if self._duration != value:
            self._duration = value
            self.updated_attribute("duration",'set')

    def set(self, dct):
        duration = dct.get('duration', None)
        if duration != None:
            self.duration = duration

        super(Delay,self).set(dct)


class Gripper(Primitive):

    '''
    Data structure methods
    '''

    def __init__(self, position=0, effort=0, speed=0, type='', name='', uuid=None,
                 parent=None, append_type=True):

        self._position = None
        self._effort = None
        self._speed = None

        super(Gripper,self).__init__(
            type='gripper.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.position = position
        self.effort = effort
        self.speed = speed

    def to_dct(self):
        msg = super(Gripper,self).to_dct()
        msg.update({
            'position': self.position,
            'effort': self.effort,
            'speed': self.speed
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            uuid=dct['uuid'],
            type=dct['type'],
            append_type=False,
            position=dct['position'],
            effort=dct['effort'],
            speed=dct['speed'])

    '''
    Data accessor/modifier methods
    '''

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        if self._position != value:
            self._position = value
            self.updated_attribute('position','set')

    @property
    def effort(self):
        return self._effort

    @effort.setter
    def effort(self, value):
        if self._effort != value:
            self._effort = value
            self.updated_attribute('effort','set')

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, value):
        if self._speed != value:
            self._speed = value
            self.updated_attribute('speed','set')

    def set(self, dct):
        position = dct.get('position', None)
        if position != None:
            self.position = position

        effort = dct.get('effort', None)
        if effort != None:
            self.effort = effort

        speed = dct.get('speed', None)
        if speed != None:
            self.speed = speed

        super(Gripper,self).set(dct)


class MachinePrimitive(Primitive):

    '''
    Data structure methods
    '''

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None, append_type=True):
        self._machine_uuid = None

        super(MachinePrimitive,self).__init__(
            type='machine-primitive.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.machine_uuid = machineUuid

    def to_dct(self):
        msg = super(MachinePrimitive,self).to_dct()
        msg.update({
            'machine_uuid': self.machine_uuid
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            uuid=dct['uuid'],
            type=dct['type'],
            append_type=False,
            machineUuid=dct['machine_uuid'])

    '''
    Data accessor/modifier methods
    '''

    @property
    def machine_uuid(self):
        return self._machine_uuid

    @machine_uuid.setter
    def machine_uuid(self, value):
        if self._machine_uuid != value:
            self._machine_uuid = value
            self.updated_attribute('machine_uuid','set')

    def set(self, dct):
        if 'machine_uuid' in dct.keys():
            self.machine_uuid = dct['machine_uuid']

        super(MachinePrimitive,self).set(dct)


class MachineStart(MachinePrimitive):

    '''
    Data structure methods
    '''

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None, append_type=True):
        super(MachineStart,self).__init__(
            machineUuid=machineUuid,
            type='machine-start.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)


class MachineWait(MachinePrimitive):

    '''
    Data structure methods
    '''

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None, append_type=True):
        super(MachineWait,self).__init__(
            machineUuid=machineUuid,
            type='machine-wait.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)


class MachineStop(MachinePrimitive):

    '''
    Data structure methods
    '''

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None, append_type=True):
        super(MachineStop,self).__init__(
            machineUuid=machineUuid,
            type='machine-stop.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)


class MachineInitialize(MachinePrimitive):

    '''
    Data structure methods
    '''

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None,
                 append_type=True):
        super(MachineInitialize,self).__init__(
            machineUuid=machineUuid,
            type='machine-initialize.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)


class Breakpoint(Primitive):

    '''
    Data structure methods
    '''

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True):
        super(Breakpoint,self).__init__(
            type='breakpoint.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)
