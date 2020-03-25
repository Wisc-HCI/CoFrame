from waypoint import Waypoint


class Location(Waypoint):

    '''
    Data structure methods
    '''

    def __init__(self, position=None, orientation=None, joints=None, type='',
                 name='', uuid=None, parent=None, append_type=True):
        super(Location,self).__init__(
            position=position,
            orientation=orientation,
            joints=joints,
            type='location.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)