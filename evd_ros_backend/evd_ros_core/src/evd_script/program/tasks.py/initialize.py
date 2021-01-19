from ..task import Task
from ..machine_operations import MachineInitialize
from ..primitives import MoveUnplanned
from .open_gripper import OpenGripper


class Initialize(Task):

    '''
    Data structure methods
    '''

    def __init__(self, homeLocUuid=None, machineUuids=[], type='', name='',
                 uuid=None, parent=None, append_type=True, primitives=None, context=None):

        if primitives == None:
            primitives = []

        primitives += [MachineInitialize(id) for id in machineUuids]

        primitives += [
            MoveUnplanned(homeLocUuid,True),
            OpenGripper()
        ]

        super(Initialize,self).__init__(
            type='initialize.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            context=context,
            primitives=primitives)