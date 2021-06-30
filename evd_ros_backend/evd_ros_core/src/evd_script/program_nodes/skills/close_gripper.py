'''
Simplified parameterization for gripper primitive to fully closed
'''

from ...data_nodes.skill_argument import SkillArgument
from ...data_nodes.thing import Thing
from ..skill import Skill
from ..primitives import Gripper


class CloseGripper(Skill):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Close Gripper'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'close-gripper' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Skill.full_type_string() + cls.type_string()

    def __init__(self,  primitives=None, arguments=None, parameters=None, type='',
                 name=None, uuid=None, parent=None, append_type=True,
                 editable=True, deleteable=True, description=''):

        thing_uuid_arg = None
        if arguments != None:
            for a in arguments:
                if a.name == 'thing_uuid':
                    thing_uuid_arg = a

            if thing_uuid_arg == None:
                thing_uuid_arg = SkillArgument(parameter_key='thing_uuid', name='thing_uuid', parameter_type=Thing.full_type_string())
                arguments.append(thing_uuid_arg)

        else: 
            thing_uuid_arg = SkillArgument(parameter_key='thing_uuid', name='thing_uuid', parameter_type=Thing.full_type_string())
            arguments = [thing_uuid_arg]

        if primitives == None:
            primitives = [
                Gripper(
                    thing_uuid=thing_uuid_arg.temporary_value,
                    semantic=Gripper.SEMANTIC_GRASPING,
                    position=100,
                    editable=editable,
                    deleteable=False)
            ]
        else:
            primitives[0].thing_uuid = thing_uuid_arg.temporary_value

        if name == None:
            name = 'Close Gripper'

        super(CloseGripper,self).__init__(
            type=CloseGripper.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description,
            parameters=parameters,
            primitives=primitives,
            arguments=arguments)
