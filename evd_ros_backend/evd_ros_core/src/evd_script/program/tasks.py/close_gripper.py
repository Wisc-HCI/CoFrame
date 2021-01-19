from ..task import Task
from ..primitives import Gripper


class CloseGripper(Task):

    '''
    Data structure methods
    '''

    def __init__(self, position=100, effort=100, speed=100, type='', name='',
                 uuid=None, parent=None, append_type=True, primitives=None, context=None):

        if primitives == None:
            primitives=[
                Gripper(
                    position=position,
                    effort=effort,
                    speed=speed)
            ]

        super(CloseGripper,self).__init__(
            type='close-gripper.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            context=context,
            primitives=primitives)
