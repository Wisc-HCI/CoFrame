'''
Machine Blocking Process simplifies the generalzied machining process for a robot. The
machining could be a pre-written CNC routine that gets invoked by this primitive, then
waits until that routine finishes, and will perform any "cleanup" at end of the process.
'''

from ..skill import Skill
from .machine_start import MachineStart
from .machine_stop import MachineStop
from .machine_wait import MachineWait


class MachineBlockingProcess(Skill):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Machine Blocking Process Skill'

'''
    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'machine-blocking-process' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Skill.full_type_string() + cls.type_string()

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None,
                 append_type=True, primitives=None, editable=True, deleteable=True,
                 description=''):

        if primitives == None:
            primitives=[
                MachineStart(machineUuid, editable=editable, deleteable=False),
                MachineWait(machineUuid, editable=editable, deleteable=False),
                MachineStop(machineUuid, editable=editable, deleteable=False)
            ]

        super(MachineBlockingProcess,self).__init__(
            type=MachineBlockingProcess.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            primitives=primitives,
            editable=editable,
            deleteable=deleteable,
            description=description)
'''