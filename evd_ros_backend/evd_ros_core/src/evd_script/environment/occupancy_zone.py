from ..node import Node

class OccupancyZone(Node):

    '''
    Constants
    '''

    HUMAN_TYPE = "human"
    ROBOT_TYPE = "robot"

    '''
    Data structure methods
    '''

    def __init__(self, occupancyType, posX = 0, posZ = 0, sclX = 1, sclZ = 1, type='', name='', parent=None, uuid=None, append_type=True):
        self._occupancy_type = None
        self._position_x = None
        self._position_z = None
        self._scale_x = None
        self._scale_z = None

        super(OccupancyZone,self).__init__(
            type='occupancy-zone.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.occupancy_type = occupancyType
        self.position_x = posX
        self.position_z = posZ
        self.scale_x = sclX
        self.scale_z = sclZ

    def to_dct(self):
        msg = super(OccupancyZone,self).to_dct()
        msg.update({
            'occupancy_type': self.occupancy_type,
            'position_x': self.position_x,
            'position_z': self.position_z,
            'scale_x': self.scale_x,
            'scale_z': self.scale_z
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(occupancyType=dct['occupancy_type'],
                   posX=dct['position_x'],
                   posZ=dct['position_z'],
                   sclX=dct['scale_x'],
                   sclZ=dct['scale_z'],
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    '''
    Data accessor/modifier methods
    '''

    @property
    def occupancy_type(self):
        return self._occupancy_type

    @occupancy_type.setter
    def occupancy_type(self, value):
        if self._occupancy_type != value:

            if value != self.ROBOT_TYPE and value != self.HUMAN_TYPE:
                raise Exception('Invalid occupancy zone type specified')

            self._occupancy_type = value
            self.updated_attribute('occupancy_type', 'set')

    @property
    def position_x(self):
        return self._position_x

    @position_x.setter
    def position_x(self, value):
        if self._position_x != value:
            self._position_x = value
            self.updated_attribute('position_x', 'set')

    @property
    def position_z(self):
        return self._position_x

    @position_z.setter
    def position_z(self, value):
        if self._position_z != value:
            self._position_z = value
            self.updated_attribute('position_z', 'set')

    @property
    def scale_x(self):
        return self._scale_x

    @scale_x.setter
    def scale_x(self, value):
        if self._scale_x != value:
            self._scale_x = value
            self.updated_attribute('scale_x', 'set')

    @property
    def scale_z(self):
        return self._scale_z

    @scale_z.setter
    def scale_z(self, value):
        if self._scale_z != value:
            self._scale_z = value
            self.updated_attribute('scale_z', 'set')

    def set(self, dct):

        if 'occupancy_type' in dct.keys():
            self.occupancy_type = dct['occupancy_type']

        if 'position_x' in dct.keys():
            self.position_x = dct['position_x']

        if 'position_z' in dct.keys():
            self.position_z = dct['position_z']

        if 'scale_x' in dct.keys():
            self.scale_x = dct['scale_x']

        if 'scale_z' in dct.keys():
            self.scale_z = dct['scale_z']

        super(OccupancyZone,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):
        super(ReachSphere,self).deep_update()

        self.updated_attribute('occupancy_type','update')
        self.updated_attribute('position_x','update')
        self.updated_attribute('position_z','update')
        self.updated_attribute('scale_x','update')
        self.updated_attribute('scale_z','update')

    def shallow_update(self):
        super(ReachSphere,self).shallow_update()

        self.updated_attribute('occupancy_type','update')
        self.updated_attribute('position_x','update')
        self.updated_attribute('position_z','update')
        self.updated_attribute('scale_x','update')
        self.updated_attribute('scale_z','update')
