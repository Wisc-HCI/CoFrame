'''
Initialize should be the first skill evoked in the program to
configure all machines and jog cobot to initial position.
'''

from ...data_nodes.skill_argument import SkillArgument
from ..skill import Skill
from ..primitives import MoveUnplanned, MachineInitialize, Gripper


def Initialize(machine_names=[], name=None, uuid=None, parent=None, editable=True, deleteable=False, description=''):

    loc_uuid_arg = SkillArgument(parameter_key='location_uuid', name='home_location_uuid')
    machine_uuid_args = [SkillArgument(parameter_key='machine_uuid', name=name) for name in machine_names]
    
    arguments = [loc_uuid_arg] + machine_uuid_args

    primitives = [MachineInitialize(a.temporary_value, editable=editable, deleteable=False) for a in machine_uuid_args]
    primitives += [
        Gripper(position=0, editable=editable, deleteable=False),
        MoveUnplanned(loc_uuid_arg.temporary_value,True, editable=editable, deleteable=False)
    ]

    return Skill(
        name= name if name != None else 'Initialize',
        uuid=uuid,
        parent=parent,
        editable=editable,
        deleteable=deleteable,
        description=description,
        arguments=arguments,
        primitives=primitives)
