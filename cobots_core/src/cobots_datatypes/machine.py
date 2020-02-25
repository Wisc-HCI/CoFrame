from node import Node


class Machine(Node):

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True):
        super(Node,self).__init__(
            type='machine.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)
