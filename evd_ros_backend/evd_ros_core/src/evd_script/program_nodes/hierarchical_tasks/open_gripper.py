'''
Simplified parameterization for gripper primitive to fully open
'''

from ..hierarchical import Hierarchical
from ..primitives import Gripper


class OpenGripper(Hierarchical):

    '''
    Data structure methods
    '''
    
    @classmethod
    def display_name(cls):
        return 'Open Gripper'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'open-gripper' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Hierarchical.full_type_string() + cls.type_string()

    def __init__(self, position=0, effort=100, speed=100, thing_uuid=None, type='',
                 name='', uuid=None, parent=None, append_type=True, primitives=None,
                 editable=True, deleteable=True, description='', parameters=None):

        if primitives == None:
            primitives=[
                Gripper(
                    thing_uuid=thing_uuid,
                    semantic=Gripper.SEMANTIC_RELEASING if thing_uuid != None else Gripper.SEMANTIC_AMBIGUOUS,
                    position=position,
                    effort=effort,
                    speed=speed,
                    editable=editable,
                    deleteable=False)
            ]

        super(OpenGripper,self).__init__(
            type=OpenGripper.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            primitives=primitives,
            editable=editable,
            deleteable=deleteable,
            description=description,
            parameters=parameters)
