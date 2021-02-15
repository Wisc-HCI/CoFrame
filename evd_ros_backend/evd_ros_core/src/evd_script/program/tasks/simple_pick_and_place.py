from ..task import Task
from ..primitives import MoveTrajectory
from .open_gripper import OpenGripper
from .close_gripper import CloseGripper


class SimplePickAndPlace(Task):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls):
        return 'simple-pick-and-place.'

    @classmethod
    def full_type_string(cls):
        return Task.full_type_string() + cls.type_string()

    def __init__(self, startLocUuid=None, pickLocUuid=None, placeLocUuid=None, thing_uuid=None,
                 type='', name='', uuid=None, parent=None, append_type=True, primitives=None):

        if primitives == None:
            primitives = [
                MoveTrajectory(startLocUuid,pickLocUuid),
                CloseGripper(thing_uuid=thing_uuid),
                MoveTrajectory(pickLocUuid,placeLocUuid),
                OpenGripper(thing_uuid=thing_uuid)
            ]

        super(SimplePickAndPlace,self).__init__(
            type='simple-pick-and-place.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            primitives=primitives)
