from ..node import Node


#TODO link to collision meshes
#TODO provide a thing input region
#TODO provide a thing output region
#TODO handle thing conversion (thing consumers and thing generators)

class Machine(Node):

    '''
    Data structure methods
    '''

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True):
        super(Machine,self).__init__(
            type='machine.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)
