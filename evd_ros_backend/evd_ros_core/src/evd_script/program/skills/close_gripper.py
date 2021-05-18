from ..skill import Skill
from ..primitives import Gripper


class CloseGripper(Skill):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'close-gripper' + '.' if trailing_delim else ''

    @classmethod
    def full_type_string(cls):
        return Skill.full_type_string() + cls.type_string()

    def __init__(self, position=100, effort=100, speed=100, thing_uuid=None, type='',
                 name='', uuid=None, parent=None, append_type=True, primitives=None):

        if primitives == None:
            primitives=[
                Gripper(
                    thing_uuid=thing_uuid,
                    position=position,
                    effort=effort,
                    speed=speed)
            ]

        super(CloseGripper,self).__init__(
            type=CloseGripper.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            primitives=primitives)
