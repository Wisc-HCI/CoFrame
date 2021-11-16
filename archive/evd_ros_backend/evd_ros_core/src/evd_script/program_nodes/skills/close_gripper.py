'''
Simplified parameterization for gripper primitive to fully closed
'''

from ...data_nodes.skill_argument import SkillArgument
from ...data_nodes.thing import Thing
from ..skill import Skill
from ..primitives import Gripper


def CloseGripper(name=None, uuid=None, parent=None, editable=True, deleteable=False, description=''):
 
    thing_uuid_arg = SkillArgument(
                        parameter_key='thing_uuid', 
                        name='thing_uuid', 
                        parameter_type=Thing.full_type_string())
    arguments = [thing_uuid_arg]

    primitives = [
        Gripper(
            thing_uuid=thing_uuid_arg.temporary_value,
            semantic=Gripper.SEMANTIC_GRASPING,
            position=100,
            editable=editable,
            deleteable=False)
    ]

    return Skill(
        name=name if name != None else 'Close Gripper',
        uuid=uuid,
        parent=parent,
        editable=editable,
        deleteable=deleteable,
        description=description,
        arguments=arguments,
        primitives=primitives)