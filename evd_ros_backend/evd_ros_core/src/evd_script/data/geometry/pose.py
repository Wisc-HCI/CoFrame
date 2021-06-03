'''
Provides a conveinent interface between low-level pose implementations (like ROS) and
the EvD AST.
'''

import geometry_msgs.msg as ros_msgs

from ...node import Node
from .position import Position
from .orientation import Orientation
from ...node_parser import NodeParser


class Pose(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(clss, trailing_delim=True):
        return 'pose' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    def __init__(self, position=None, orientation=None, type='', name='', uuid=None,
                 parent=None, append_type=True, editable=True, deleteable=True,
                 description=''):

        self._position = None
        self._orientation = None

        super(Pose,self).__init__(
            type=Pose.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        if position is None:
            self.position = Position(0,0,0)
        else:
            self.position = position

        if orientation is None:
            self.orientation = Orientation(0,0,0,1)
        else:
            self.orientation = orientation

    def to_dct(self):
        msg = super(Pose,self).to_dct()
        msg.update({
            'position': self.position.to_dct(),
            'orientation': self.orientation.to_dct()
        })
        return msg

    def to_simple_dct(self):
        return {
            'position': self.position.to_simple_dct(),
            'orientation': self.orientation.to_simple_dct()
        }

    def to_ros(self):
        return ros_msgs.Pose(position=self.position.to_ros(),
                    orientation=self.orientation.to_ros())

    @classmethod
    def from_dct(cls, dct):
        return cls(position=NodeParser(dct['position'], enforce_types=[Position.type_string(trailing_delim=False)]),
                   orientation=NodeParser(dct['orientation'], enforce_types=[Orientation.type_string(trailing_delim=False)]),
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    @classmethod
    def from_simple_dct(cls, dct):
        return cls(position=Position.from_simple_dct(dct['position']),
                   orientation=Orientation.from_simple_dct(dct['orientation']))

    @classmethod
    def from_ros(cls, obj):
        return cls(position=Position.from_ros(obj.position),
                   orientation=Orientation.from_ros(obj.orientation))

    '''
    Data accessor/modifier methods
    '''

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        if self._position != value:
            if value == None:
                raise Exception('Position cannot be None')

            if self._position != None:
                self._position.remove_from_cache()

            self._position = value
            self._position.parent = self
            self.updated_attribute('position','set')

    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        if self._orientation != value:
            if value == None:
                raise Exception('Orientation cannot be None')

            if self._orientation != None:
                self._orientation.remove_from_cache()

            self._orientation = value
            self._orientation.parent = self
            self.updated_attribute('orientation','set')

    def set(self, dct):
        pos = dct.get('position',None)
        if pos != None:
            self.position = NodeParser(pos, enforce_types=[Position.type_string(trailing_delim=False)])

        ort = dct.get('orientation',None)
        if ort != None:
            self.orientation = NodeParser(ort, enforce_types=[Orientation.type_string(trailing_delim=False)])

        super(Pose,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):
        self.position.remove_from_cache()
        self.orientation.remove_from_cache()

        super(Pose,self).remove_from_cache()

    def add_to_cache(self):
        self.position.add_to_cache()
        self.orientation.add_to_cache()

        super(Pose,self).add_to_cache()

    '''
    Update Methods
    '''

    def late_construct_update(self):
        self.orientation.late_construct_update()
        self.position.late_construct_update()

        super(Pose,self).late_construct_update()

    def deep_update(self):
        self.orientation.deep_update()
        self.position.deep_update()

        super(Pose,self).deep_update()

        self.updated_attribute('position','update')
        self.updated_attribute('orientation','update')

    def shallow_update(self):
        super(Pose,self).shallow_update()

        self.updated_attribute('position','update')
        self.updated_attribute('orientation','update')
