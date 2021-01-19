from ..task import Task
from ..primitives import MoveTrajectory
from .open_gripper import OpenGripper
from .close_gripper import CloseGripper


class SimplePickAndPlace(Task):

    '''
    Data structure methods
    '''

    def __init__(self, startLocUuid=None, pickLocUuid=None, placeLocUuid=None, type='', name='',
                 uuid=None, parent=None, append_type=True, create_default=True, primitives=None, context=None):

        if primitives == None:
            primitives = [
                MoveTrajectory(startLocUuid,pickLocUuid,create_default=create_default),
                CloseGripper(),
                MoveTrajectory(pickLocUuid,placeLocUuid,create_default=create_default),
                OpenGripper()
            ]

        super(PickAndPlace,self).__init__(
            type='simple-pick-and-place.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            context=context,
            primitives=primitives)
