'''
Joints provides an abstraction on joint information for a robot
'''

from ...type_defs import BOOLEAN_TYPE, NUMBER_TYPE, STRING_TYPE
from ...node import Node
from sensor_msgs.msg import JointState


class Joints(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Joints'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'joints' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Node.template()
        template['fields'].append({
            'type': NUMBER_TYPE,
            'key': 'length',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': NUMBER_TYPE,
            'key': 'joint_positions',
            'is_uuid': False,
            'is_list': True
        })
        template['fields'].append({
            'type': STRING_TYPE,
            'key': 'joint_names',
            'is_uuid': False,
            'is_list': True
        })
        template['fields'].append({
            'type': BOOLEAN_TYPE,
            'key': 'reachable',
            'is_uuid': False,
            'is_list': False
        })
        return template

    def __init__(self, length=6, joint_positions=None, reachable=False, joint_names=None, 
                 type='', name='', uuid=None, parent=None, append_type=True, editable=True, 
                 deleteable=True, description=''):
        self._length = None
        self._joint_positions = None
        self._joint_names = None
        self._reachable = None

        super(Joints,self).__init__(
            type=Joints.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        self.length = length
        self.joint_positions = joint_positions
        self.reachable = reachable
        self.joint_names = joint_names

    def to_dct(self):
        msg = super(Joints,self).to_dct()
        msg.update({
            'joint_positions': self.joint_positions,
            'joint_names': self.joint_names,
            'reachable': self.reachable,
            'length': self.length
        })

    def to_lists(self):
        return self.joint_positions, self.joint_names

    def to_ros(self):
        msg = JointState()
        msg.name = self.joint_names if self.joint_names != None else []
        msg.position = self.joint_positions if self.joint_positions != None else []
        msg.velocity = [0 for _ in range(0,len(msg.position))]
        msg.effort = [0 for _ in range(0,len(msg.position))]
        return msg

    def to_joint_dct(self):
        if self.positions == None or self.joint_names == None:
            raise Exception('both positions and joint_names must be lists in order to produce dct')

        dct = {}
        for i in range(0,len(self.positions)):
            dct[self.joint_names[i]] = self.positions[i]

        return dct

    @classmethod
    def from_dct(cls, dct):
        return cls(
            length=dct['length'],
            joint_positions=dct['joint_positions'],
            reachable=dct['reachable'],
            joint_names=dct['joint_names'],
            type=dct['type'] if 'type' in dct.keys() else '',
            append_type=not 'type' in dct.keys(),
            editable=dct['editable'],
            deleteable=dct['deleteable'],
            description=dct['description'],
            uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
            name=dct['name'] if 'name' in dct.keys() else '')
    
    @classmethod
    def from_lists(cls, positions, names=None):
        reachable = positions != None and len(positions) > 0

        if names != None and positions != None and len(names) != len(positions):
            raise Exception('Optional name vector must be of same length as position vector')

        return cls(
            length=len(positions),
            positions= positions if reachable else None,
            reachable= reachable,
            joint_names= names)

    @classmethod
    def from_ros(cls, msg):
        return cls(
            length=len(msg.position),
            positions= msg.position,
            joint_names= msg.name) 

    @classmethod
    def from_joint_dct(cls, dct):
        return cls(
            length=len(dct.keys()),
            positions= dct.values(),
            joint_names=dct.keys())

    @classmethod
    def Unknown(cls, length=None):
        positions = (length if length != None else 1) * ['?']
        names = (length if length != None else 1) * ['?']
        return cls(length=length, joint_positions=positions, joint_names=names)

    '''
    Data accessor/modifier methods
    '''

    @property
    def length(self):
        return self._length

    @length.setter
    def length(self, value):
        if value != self._length:
            if self._joint_positions != None or self._joint_names != None:
                raise Exception('Cannot set length expectation unless both position and name vectors are none')
            elif not isinstance(value,int):
                raise Exception('length must be an integer')
            elif value < 0:
                raise Exception('length must be at least zero')

            self._length = value
            self.updated_attribute('length','set')

    @property
    def joint_positions(self):
        return self._joint_positions

    @joint_positions.setter
    def joint_positions(self, value):
        if self._joint_positions != value:
            if value != None and self.length != None and len(value) != self.length:
                raise Exception('vector must match length expectation')

            self._joint_positions = value
            self.updated_attribute('joint_positions','set')

            if self._joint_positions == None:
                self.reachable = False

    @property
    def joint_names(self):
        return self._joint_names

    @joint_names.setter
    def joint_names(self, value):
        if value != self._joint_names:
            if value != None and self.length != None and len(value) != self.length:
                raise Exception('vector must match length expectation')

            self._joint_names = value
            self.updated_attribute('joint_names','set')

    @property
    def reachable(self):
        # If positions are None and this is true then it is just as valid as if it were false
        return self._reachable

    @reachable.setter
    def reachable(self, value):
        if value != self._reachable:
            self._reachable = value
            self.updated_attribute('reachable','set')

    def set_joint_positions_by_names(self, positions, names):
        for i, n in enumerate(names):
            for j, name in enumerate(self._joint_names):
                if name == n:
                    self._joint_names[j] = positions[i]
                    break

    def get_joint_positions_by_names(self, names):
        joints = []

        for n in names:
            position = None

            for j, name in enumerate(self._joint_names):
                if name == n:
                    position = self._joint_names[j]
                    break

            if position != None:
                joints.append(position)
        
        return joints

    def set(self, dct):

        if 'length' in dct.keys():
            # bypass check as overriding length property
            self._length = dct['length']
            self.updated_attribute('length','set')

        if 'joint_positions' in dct.keys():
            self.joint_positions = dct['joint_positions']

        if 'joint_names' in dct.keys():
            self.joint_names = dct['joint_names']

        if 'reachable' in dct.keys():
            self.reachable = dct['reachable']

        super(Joints,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):
        super(Joints,self).deep_update()

        self.updated_attribute('length','update')
        self.updated_attribute('reachable','update')
        self.updated_attribute('joint_positions','update')
        self.updated_attribute('joint_names','update')

    def shallow_update(self):
        super(Joints,self).shallow_update()

        self.updated_attribute('length','update')
        self.updated_attribute('reachable','update')
        self.updated_attribute('joint_positions','update')
        self.updated_attribute('joint_names','update')