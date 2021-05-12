from abc import ABC, abstractmethod

from ....node import Node


class AbstractConditional(Node,ABC):

    '''
    Data structure methods
    '''

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True):
        super(AbstractConditional,self).__init__(
            type='conditional.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

    @abstractmethod
    def get_state():
        return False #must return a boolean result here
