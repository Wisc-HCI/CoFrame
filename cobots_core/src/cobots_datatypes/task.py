from primitive import *
from trajectory import Trajectory


class Task(Primitive):

    def __init__(self, primitives=[], type='', name='', uuid=None, parent=None,
                 append_type=True):
        super(Primitive,self).__init__(
            type='task.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self._initialize_private_members()

        self.primitives = primitives

    def _initialize_private_members(self):
        self._primitives = None

    @property
    def primitives(self):
        return self._primitives

    @primitives.setter
    def primitives(self, value):
        if self._primitives != value:
            self._primitives = value
            for p in self._primitives:
                p.parent = self

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('primitives')])

    def add_primitive(self, prm):
        prm.parent = self
        self._primitives.append(prm)

    def reorder_primitives(self, uuid, shift):
        idx = None
        for i in range(0,len(self._primitives)):
            if self._primitives[i].uuid == uuid:
                idx = i
                break

        if idx != None:
            shiftedIdx = idx + shift
            if shiftedIdx < 0 or shiftedIdx >= len(self._primitives):
                raise Exception("Index out of bounds")

            copy = self._primitives.pop(idx)
            self._primitives.insert(shiftedIdx,copy) #TODO check to make sure not off by one

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('primitives')])

    def delete_primitive(self, uuid):
        delIdx = None
        for i in range(0,len(self._primitives)):
            if self._primitives[i].uuid == uuid:
                delIdx = i
                break

        if delIdx != None:
            self._primitives.pop(i)
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('primitives')])

    def get_primitive(self, uuid):
        for p in self.primitives:
            if p.uuid == uuid:
                return p
        return None

    def to_dct(self):
        msg = super(Primitive,self).to_dct()
        msg.update({
            'primitives': [p.to_dct() for p in self.primitives]
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            uuid=dct['uuid'],
            primitives=dct['primitives']
        )


class CloseGripper(Task):

    def __init__(self, position=0, effort=100, speed=100, type='', name='',
                 uuid=None, parent=None, append_type=True):
        super(Task,self).__init__(
            type='close-gripper.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            primitives=[
                Gripper(
                    position=position,
                    effort=effort,
                    speed=speed)
            ])


class OpenGripper(Task):

    def __init__(self, position=0, effort=100, speed=100, type='', name='',
                 uuid=None, parent=None, append_type=True):
        super(Task,self).__init__(
            type='open-gripper.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            primitives=[
                Gripper(
                    position=position,
                    effort=effort,
                    speed=speed)
            ])


class PickAndPlace(Task):

    def __init__(self, startLocUuid=None, pickLocUuid=None, placeLocUuid=None, type='', name='',
                 uuid=None, parent=None, append_type=True):
        super(Task,self).__init__(
            type='pick-and-place.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        trajPick = Trajectory(startLocUuid,pickLocUuid)
        trajPlace = Trajectory(pickLocUuid,placeLocUuid)

        self.primitives = [
            MoveTrajectory(startLocUuid,pickLocUuid,[trajPick],trajPick.uuid),
            CloseGripper()
            MoveTrajectory(pickLocUuid,placeLocUuid,[trajPlace],trajPlace.uuid),
            OpenGripper()
        ]


class Initialize(Task):

    def __init__(self, homeLocUuid=None, machineUuid=None, type='', name='',
                 uuid=None, parent=None, append_type=True):
        super(Task,self).__init__(
            type='initialize.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            primitives=[
                MoveUnplanned(homeLocUuid,True),
                OpenGripper(),
                MachineInitialize(machineUuid)
            ])


class MachineBlockingProcess(Task):

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None,
                 append_type=True):
        super(Task,self).__init__(
            type='machine-blocking-process.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            primitives=[
                MachineStart(machineUuid),
                MachineWait(machineUuid),
                MachineStop(machineUuid)
            ])


class Loop(Task):

    def __init__(self, primitives=[], type='', name='', uuid=None, parent=None,
                 append_type=True):
        super(Task,self).__init__(
            type='task.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            primitives=primitives)
