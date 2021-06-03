'''
Pick-and-place is a very common activity for collaborative robots.

This is a simplified (non-conditional/error-handling) implementation that
commands an end-effector to move to a target object, grasp it, move to final
location, and then release it.
'''

from ..skill import Skill
from ..primitives import MoveTrajectory
from .open_gripper import OpenGripper
from .close_gripper import CloseGripper


class SimplePickAndPlace(Skill):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'simple-pick-and-place' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Skill.full_type_string() + cls.type_string()

    def __init__(self, startLocUuid=None, pickLocUuid=None, placeLocUuid=None, thing_uuid=None,
                 type='', name='', uuid=None, parent=None, append_type=True, primitives=None, editable=True, deleteable=True):

        if primitives == None:
            primitives = [
                MoveTrajectory(startLocUuid,pickLocUuid, editable=editable, deleteable=deleteable),
                CloseGripper(thing_uuid=thing_uuid, editable=editable, deleteable=deleteable),
                MoveTrajectory(pickLocUuid,placeLocUuid, editable=editable, deleteable=deleteable),
                OpenGripper(thing_uuid=thing_uuid, editable=editable, deleteable=deleteable)
            ]

        super(SimplePickAndPlace,self).__init__(
            type=SimplePickAndPlace.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            primitives=primitives,
            editable=editable,
            deleteable=deleteable)
