from ..node import Node
from ..data.trajectory import Trajectory


class Primitive(Node):

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True):
        super(Primitive,self).__init__(
            type='primitive.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        def refresh_cache(self):
            self.add_to_cache(self.uuid,self)
            super(Primitive,self).refresh_cache()


class MoveTrajectory(Primitive):

    def __init__(self, startLocUuid=None, endLocUuid=None, trajectories=[],
                 runnableTrajUuid=None, type='', name='', uuid=None, parent=None,
                 append_type=True, create_default=True):
        super(MoveTrajectory,self).__init__(
            type='move-trajectory.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self._start_location_uuid = None
        self._end_location_uuid = None
        self._trajectories = []
        self._runnable_trajectory_uuid = None

        self.start_location_uuid = startLocUuid
        self.end_location_uuid = endLocUuid
        self.trajectories = trajectories
        self.runnable_trajectory_uuid = runnableTrajUuid

        if len(self.trajectories) == 0 and create_default:
            self.add_trajectory(Trajectory(self.start_location_uuid,self._end_location_uuid))

    @property
    def start_location_uuid(self):
        return self._start_location_uuid

    @start_location_uuid.setter
    def start_location_uuid(self, value):
        if self._start_location_uuid != value:
            self._start_location_uuid = value

            for t in self.trajectories:
                t.start_location_uuid = self._start_location_uuid

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('start_location_uuid','set')])

    @property
    def end_location_uuid(self):
        return self._end_location_uuid

    @end_location_uuid.setter
    def end_location_uuid(self, value):
        if self._end_location_uuid != value:
            self._end_location_uuid = value

            for t in self.trajectories:
                t.end_location_uuid = self._end_location_uuid

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('end_location_uuid','set')])

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
                self.add_to_cache(t.uuid,t)

            runnableFound = False
            for t in self._trajectories:
                t.parent = self
                if t.uuid == self.runnable_trajectory_uuid:
                    runnableFound = True

            if not runnableFound:
                self.runnable_trajectory_uuid = None

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('trajectories','set')])

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

            self._runnable_trajectory_uuid = value

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('runnable_trajectory_uuid','set')])

    def add_trajectory(self, t):
        t.parent = self
        self._trajectories.append(t)
        self.add_to_cache(t.uuid,t)

        if len(self._trajectories) == 1:
            self.runnable_trajectory_uuid = t.uuid

        if self._parent != None:
            self._parent.child_changed_event(
                [self._child_changed_event_msg('trajectories','add')])

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

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('trajectories','reorder')])

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

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('trajectories','delete')])

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

    def remove_from_cache(self):
        for t in self._trajectories:
            t.remove_from_cache()

        super(MoveTrajectory,self).remove_from_cache()

    def set(self, dct):
        pass #TODO write this

    def delete_child(self, uuid):
        pass #TODO write this

    def refresh_cache(self):
        for t in self._trajectories:
            t.refresh_cache()

        super(MoveTrajectory,self).refresh_cache()


class MoveUnplanned(Primitive):

    def __init__(self, locUuid, manual_safety=True, type='', name='', uuid=None, parent=None, append_type=True):
        super(MoveUnplanned,self).__init__(
            type='move-unplanned.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self._location_uuid = None
        self._manual_safety = None

        self.manual_safety = manual_safety
        self.location_uuid = locUuid

    @property
    def manual_safety(self):
        return self._manual_safety

    @manual_safety.setter
    def manual_safety(self, value):
        if self._manual_safety != value:
            self._manual_safety = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('manual_safety','set')])

    @property
    def location_uuid(self):
        return self._location_uuid

    @location_uuid.setter
    def location_uuid(self, value):
        if self._location_uuid != value:
            self._location_uuid = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('location_uuid','set')])

    def to_dct(self):
        msg = super(MoveUnplanned,self).to_dct()
        msg.update({
            'location_uuid': self.location_uuid,
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            type=dct['type'],
            append_type=False,
            uuid=dct['uuid'],
            locUuid=dct['location_uuid'])

    def set(self, dct):
        pass #TODO write this


class Delay(Primitive):

    def __init__(self, duration=0, type='', name='', uuid=None, parent=None, append_type=True):
        super(Delay,self).__init__(
            type='delay.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self._initialize_private_members()

        self.duration = duration

    def _initialize_private_members(self):
        self._duration = 0

    @property
    def duration(self):
        return self._duration

    @duration.setter
    def duration(self):
        if self._duration != duration:
            self._duration = duration
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('duration','set')])

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

    def set(self, dct):
        pass #TODO write this


class Gripper(Primitive):

    def __init__(self, position=0, effort=0, speed=0, type='', name='', uuid=None, parent=None, append_type=True):
        super(Gripper,self).__init__(
            type='gripper.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self._initialize_private_members()

        self.position = position
        self.effort = effort
        self.speed = speed

    def _initialize_private_members(self):
        self._position = None
        self._effort = None
        self._speed = None

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        if self._position != value:
            self._position = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('position','set')])

    @property
    def effort(self):
        return self._effort

    @effort.setter
    def effort(self, value):
        if self._effort != value:
            self._effort = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('effort','set')])

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, value):
        if self._speed != value:
            self._speed = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('speed','set')])

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

    def set(self, dct):
        pass #TODO write this


class MachinePrimitive(Primitive):

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None, append_type=True):
        super(MachinePrimitive,self).__init__(
            type='machine.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self._initialize_private_members()

        self.machine_uuid = machineUuid

    def _initialize_private_members(self):
        self._machine_uuid = None

    @property
    def machine_uuid(self):
        return self._machine_uuid

    @machine_uuid.setter
    def machine_uuid(self, value):
        if self._machine_uuid != value:
            self._machine_uuid = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('machine_uuid','set')])

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

    def set(self, dct):
        pass #TODO write this


class MachineStart(MachinePrimitive):

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None, append_type=True):
        super(MachineStart,self).__init__(
            machineUuid=machineUuid,
            type='machine-start.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)


class MachineWait(MachinePrimitive):

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None, append_type=True):
        super(MachineWait,self).__init__(
            machineUuid=machineUuid,
            type='machine-wait.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)


class MachineStop(MachinePrimitive):

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None, append_type=True):
        super(MachineStop,self).__init__(
            machineUuid=machineUuid,
            type='machine-stop.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)


class MachineInitialize(MachinePrimitive):

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

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True):
        super(Breakpoint,self).__init__(
            type='breakpoint.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            type=dct['type'],
            append_type=False,
            uuid=dct['uuid'])
