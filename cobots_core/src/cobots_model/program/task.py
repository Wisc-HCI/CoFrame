from primitive import *
from context import Context
from primitive_parser import PrimitiveParser


class Task(Primitive):

    def __init__(self, primitives=[], type='', name='', uuid=None, parent=None,
                 append_type=True, context=None):
        super(Task,self).__init__(
            type='task.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        if context != None:
            self._context = context
            self._context.parent_node = self
            if self.parent != None:
                self._context.parent_context = self.parent.context
            else:
                self._context.parent_context = None
        else:
            self._context = Context(
                parent_context=self.parent.context if self.parent != None else None,
                parent_node=self)

        self._primitives = []
        self.primitives = primitives

    @property
    def context(self):
        return self._context

    @property
    def primitives(self):
        return self._primitives

    @primitives.setter
    def primitives(self, value):
        if self._primitives != value:
            for p in self._primitives:
                p.remove_from_cache()

            self._primitives = value
            for p in self._primitives:
                p.parent = self
                self.add_to_cache(p.uuid,p)

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('primitives','set')])

    def add_primitive(self, prm):
        prm.parent = self
        self._primitives.append(prm)
        self.add_to_cache(prm.uuid,prm)
        if self._parent != None:
            self._parent.child_changed_event(
                [self._child_changed_event_msg('primitives','add')])

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
                    [self._child_changed_event_msg('primitives','reorder')])

    def delete_primitive(self, uuid):
        delIdx = None
        for i in range(0,len(self._primitives)):
            if self._primitives[i].uuid == uuid:
                delIdx = i
                break

        if delIdx != None:
            self._primitives.pop(i).remove_from_cache()
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('primitives','delete')])

    def get_primitive(self, uuid):
        for p in self.primitives:
            if p.uuid == uuid:
                return p
        return None

    def to_dct(self):
        msg = super(Task,self).to_dct()
        msg.update({
            'primitives': [p.to_dct() for p in self.primitives],
            'context': self.context.to_dct()
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            type=dct['type'],
            append_type=False,
            uuid=dct['uuid'],
            primitives=[PrimitiveParser(p) for p in dct['primitives']],
            context=Context.from_dct(dct['context']))

    def refresh_cache(self):
        for p in self._primitives:
            p.refresh_cache()

        self.context.refresh_cache()

        super(Task,self).refresh_cache()

    def remove_from_cache(self):
        for p in self._primitives:
            p.remove_from_cache()

        self.context.remove_from_cache()

        super(Task,self).remove_from_cache()

    def set(self, dct):
        pass #TODO write this

    def delete_child(self, uuid):
        pass #TODO write this (either primitive or context)


class CloseGripper(Task):

    def __init__(self, position=0, effort=100, speed=100, type='', name='',
                 uuid=None, parent=None, append_type=True, primitives=None, context=None):
        if primitives == None:
            primitives=[
                Gripper(
                    position=position,
                    effort=effort,
                    speed=speed)
            ]

        super(CloseGripper,self).__init__(
            type='close-gripper.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            context=context,
            primitives=primitives)


class OpenGripper(Task):

    def __init__(self, position=0, effort=100, speed=100, type='', name='',
                 uuid=None, parent=None, append_type=True, primitives=None, context=None):
        if primitives == None:
            primitives=[
                Gripper(
                    position=position,
                    effort=effort,
                    speed=speed)
            ]

        super(OpenGripper,self).__init__(
            type='open-gripper.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            context=context,
            primitives=primitives)


class PickAndPlace(Task):

    def __init__(self, startLocUuid=None, pickLocUuid=None, placeLocUuid=None, type='', name='',
                 uuid=None, parent=None, append_type=True, create_default=True, primitives=None, context=None):
        if primitives == None:
            primitives = [
                MoveTrajectory(startLocUuid,pickLocUuid,create_default=create_default),
                CloseGripper(),
                MoveTrajectory(pickLocUuid,placeLocUuid,create_default=create_default),
                OpenGripper()
            ]

        super(PickAndPlace,self).__init__(
            type='pick-and-place.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            context=context,
            primitives=primitives)


class Initialize(Task):

    def __init__(self, homeLocUuid=None, machineUuid=None, type='', name='',
                 uuid=None, parent=None, append_type=True, primitives=None, context=None):
        if primitives == None:
            primitives=[
                MoveUnplanned(homeLocUuid,True),
                OpenGripper(),
                MachineInitialize(machineUuid)
            ]

        super(Initialize,self).__init__(
            type='initialize.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            context=context,
            primitives=primitives)


class MachineBlockingProcess(Task):

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None,
                 append_type=True, primitives=None, context=None):
        if primitives == None:
            primitives=[
                MachineStart(machineUuid),
                MachineWait(machineUuid),
                MachineStop(machineUuid)
            ]

        super(MachineBlockingProcess,self).__init__(
            type='machine-blocking-process.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            context=context,
            primitives=primitives)


class Loop(Task):

    def __init__(self, primitives=[], type='', name='', uuid=None, parent=None,
                 append_type=True, context=None):
        super(Loop,self).__init__(
            type='task.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            context=context,
            primitives=primitives)
