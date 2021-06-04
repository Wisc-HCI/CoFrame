'''
Initialize should be the first skill evoked in the program to
configure all machines and jog cobot to initial position.
'''

from ..hierarchical import Hierarchical
from ..machine_operations import MachineInitialize
from ..primitives import MoveUnplanned
from .open_gripper import OpenGripper


class Initialize(Hierarchical):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'initialize' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Hierarchical.full_type_string() + cls.type_string()

    def __init__(self, homeLocUuid=None, machineUuids=[], type='', name='',
                 uuid=None, parent=None, append_type=True, primitives=None,
                 editable=True, deleteable=True, description=''):

        if primitives == None:
            primitives = []

            primitives += [MachineInitialize(id, editable=editable, deleteable=False) for id in machineUuids]

            primitives += [
                MoveUnplanned(homeLocUuid,True, editable=editable, deleteable=False),
                OpenGripper(editable=editable, deleteable=False)
            ]

        super(Initialize,self).__init__(
            type=Initialize.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            primitives=primitives,
            editable=editable,
            deleteable=deleteable,
            description=description)

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        next = super(Initialize,self).symbolic_execution(hooks)

        # Since we are initializing state, the ambigous movement is expected and does not
        # warrant special consideration
        hooks.tokens['robot']['state']['gripper']['ambiguous_flag'] = False

        return next

    def realtime_execution(self, hooks):
        next = super(Initialize,self).realtime_execution(hooks)

        # Since we are initializing state, the ambigous movement is expected and does not
        # warrant special consideration
        hooks.tokens['robot']['state']['gripper']['ambiguous_flag'] = False

        return next
