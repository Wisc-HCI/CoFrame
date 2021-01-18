from task import Task
from primitive import Primitive
from conditionals.abstract import AbstractConditional

#TODO machine conditional


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

    '''
    Update Methods
    '''

    def deep_update(self):
        super(MachinePrimitive,self).deep_update()

        self.updated_attribute('machine_uuid','update')

    def shallow_update(self):
        super(MachinePrimitive,self).shallow_update()

        self.updated_attribute('machine_uuid','update')


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


class MachineConditional(MachinePrimitive, AbstractConditional):

    '''
    Data structure methods
    '''

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None, append_type=True):
        pass #TODO
