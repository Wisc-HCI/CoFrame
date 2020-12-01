from primitive import *
from context import Context
from ..utility_functions import NodeParser


class Task(Primitive):

    '''
    Data structure methods
    '''

    def __init__(self, primitives=[], type='', name='', uuid=None, parent=None,
                 append_type=True, context=None):

        self._primitives = []
        self._context = Context()

        super(Task,self).__init__(
            type='task.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        if context != None:
            self.context = context

        self.primitives = primitives

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
            primitives=[NodeParser(p) for p in dct['primitives']],
            context=Context.from_dct(dct['context']))

    '''
    Data accessor/modifier methods
    '''

    @property
    def context(self):
        return self._context

    @context.setter
    def context(self, value):
        if self._context != value:
            self._context.remove_from_cache()

            self._context = value
            self._context.parent = self
            if self.parent != None:
                self._context.parent_context = self.parent.context

            self.updated_attribute('context','set')

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

            self.updated_attribute('primitives','set')

    def add_primitive(self, prm):
        prm.parent = self
        self._primitives.append(prm)
        self.updated_attribute('primitives','set')

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

            self.updated_attribute('primitives','reorder')

    def delete_primitive(self, uuid):
        delIdx = None
        for i in range(0,len(self._primitives)):
            if self._primitives[i].uuid == uuid:
                delIdx = i
                break

        if delIdx != None:
            self._primitives.pop(i).remove_from_cache()
            self.updated_attribute('primitives','delete')

    def get_primitive(self, uuid):
        for p in self.primitives:
            if p.uuid == uuid:
                return p
        return None

    def set(self, dct):
        if 'primitives' in dct.keys():
            self.primitives = [NodeParser(p) for p in dct['primitives']]

        if 'context' in dct.keys():
            self.context = Context.from_dct(dct['context'])

        super(Task,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):
        for p in self._primitives:
            p.remove_from_cache()

        self.context.remove_from_cache()

        super(Task,self).remove_from_cache()

    def add_to_cache(self):
        for p in self._primitives:
            p.add_to_cache()

        self.context.add_to_cache()

        super(Task,self).add_to_cache()

    '''
    Children methods
    '''

    def delete_child(self, uuid):
        success = True

        if uuid == self.context.uuid:
            self.context = Context()  # Must always have a context with a task
        elif uuid in [p.uuid for p in self.primitives]:
            self.delete_primitive(uuid)
        else:
            success = False

        return success

    '''
    Update Methods
    '''

    def deep_update(self):

        for p in self.primitives:
            p.deep_update()

        self.context.deep_update()

        super(Task,self).deep_update()

        self.updated_attribute('primitives','update')
        self.updated_attribute('context','update')

    def shallow_update(self):
        super(Task,self).shallow_update()

        self.updated_attribute('primitives','update')
        self.updated_attribute('context','update')


class CloseGripper(Task):

    '''
    Data structure methods
    '''

    def __init__(self, position=100, effort=100, speed=100, type='', name='',
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

    '''
    Data structure methods
    '''

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

    '''
    Data structure methods
    '''

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

    '''
    Data structure methods
    '''

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

    '''
    Data structure methods
    '''

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

    '''
    Data structure methods
    '''

    def __init__(self, primitives=[], type='', name='', uuid=None, parent=None,
                 append_type=True, context=None):
        super(Loop,self).__init__(
            type='loop.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            context=context,
            primitives=primitives)
