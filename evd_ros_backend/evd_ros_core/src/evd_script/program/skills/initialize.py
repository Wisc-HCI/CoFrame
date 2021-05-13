from ..skill import Skill
from ..machine_operations import MachineInitialize
from ..primitives import MoveUnplanned
from .open_gripper import OpenGripper


class Initialize(Skill):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls):
        return 'initialize.'

    @classmethod
    def full_type_string(cls):
        return Skill.full_type_string() + cls.type_string()

    def __init__(self, homeLocUuid=None, machineUuids=[], type='', name='',
                 uuid=None, parent=None, append_type=True, primitives=None):

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
            primitives=primitives)