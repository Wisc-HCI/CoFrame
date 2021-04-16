from ..skill import Skill
from .machine_start import MachineStart
from .machine_stop import MachineStop
from .machine_wait import MachineWait


class MachineBlockingProcess(Skill):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls):
        return 'machine-blocking-process.'

    @classmethod
    def full_type_string(cls):
        return Skill.full_type_string() + cls.type_string()

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None,
                 append_type=True, primitives=None):

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
            primitives=primitives)
