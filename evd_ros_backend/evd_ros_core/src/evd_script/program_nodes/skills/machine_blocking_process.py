'''
Machine Blocking Process simplifies the generalzied machining process for a robot. The
machining could be a pre-written CNC routine that gets invoked by this primitive, then
waits until that routine finishes, and will perform any "cleanup" at end of the process.
'''

from ...data_nodes.skill_argument import SkillArgument
from ..skill import Skill
from ..primitives.machine_start import MachineStart
from ..primitives.machine_stop import MachineStop
from ..primitives.machine_wait import MachineWait


def MachineBlockingProcess(name=None, uuid=None, parent=None, editable=True, deleteable=False, description=''):

    machine_uuid_arg = SkillArgument(parameter_key='machine_uuid', name='machine_uuid')
    arguments = [machine_uuid_arg]

    primitives=[
        MachineStart(machine_uuid=machine_uuid_arg.temporary_value, editable=editable, deleteable=False),
        MachineWait(machine_uuid=machine_uuid_arg.temporary_value, editable=editable, deleteable=False),
        MachineStop(machine_uuid=machine_uuid_arg.temporary_value, editable=editable, deleteable=False)
    ]

    return Skill(
        name= name if name != None else 'Machine Blocking Process',
        uuid=uuid,
        parent=parent,
        editable=editable,
        deleteable=deleteable,
        description=description,
        arguments=arguments,
        primitives=primitives)