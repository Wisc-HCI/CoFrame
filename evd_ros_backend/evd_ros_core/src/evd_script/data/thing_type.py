'''
ThingType defines a type system for things. This is useful for generating things at
run-time and for token processing during planning/simulation.
'''

from ..node import Node


class ThingType(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'thing-type' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()
    
    def __init__(self, type_name='', is_safe=True, weight=0, mesh_id=None,
                 type='', name='', parent=None, uuid=None, append_type=True):

        self._type_name = None
        self._mesh_id = None
        self._is_safe = None
        self._weight = None
        
        super(ThingType,self).__init__(
            type=ThingType.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.type_name = type_name
        self.mesh_id = mesh_id
        self.is_safe = is_safe
        self.weight = weight

    def to_dct(self):
        msg = super(ThingType,self).to_dct()
        msg.update({
            'type_name': self.type_name,
            'mesh_id': self.mesh_id,
            'is_safe': self.is_safe,
            'weight': self.weight
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(thing_type=dct['thing_type'],
                   mesh_id=dct['mesh_id'],
                   is_safe=dct['is_safe'],
                   weight=dct['weight'],
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    '''
    Data accessor/modifier methods
    '''

    @property
    def type_name(self):
        return self._type_name

    @type_name.setter
    def type_name(self, value):
        if self._type_name != value:
            self._type_name = value
            self.updated_attribute('type_name','set')

    @property
    def is_safe(self):
        return self._is_safe

    @is_safe.setter
    def is_safe(self, value):
        if self._is_safe != value:
            self._is_safe = value
            self.updated_attribute('is_safe','set')

    @property
    def weight(self):
        return self._weight

    @weight.setter
    def weight(self, value):
        if self._weight != value:
            self._weight = value
            self.updated_attribute('weight','set')

    @property
    def mesh_id(self):
        return self._mesh_id

    @mesh_id.setter
    def mesh_id(self, value):
        if self._mesh_id != value:
            self._mesh_id = value
            self.updated_attribute('mesh_id','set')

    def set(self, dct):

        if 'type_name' in dct.key():
            self.type_name = dct['type_name']

        if 'is_safe' in dct.keys():
            self.is_safe = dct['is_safe']

        if 'weight' in dct.keys():
            self.weight = dct['weight']

        if 'mesh_id' in dct.keys():
            self.mesh_id = dct['mesh_id']

        super(ThingType,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):

        super(ThingType,self).deep_update()

        self.updated_attribute('type_name','update')
        self.updated_attribute('is_safe','update')
        self.updated_attribute('weight','update')
        self.updated_attribute('mesh_id','update')

    def shallow_update(self):
        super(ThingType,self).shallow_update()

        self.updated_attribute('type_name','update')
        self.updated_attribute('is_safe','update')
        self.updated_attribute('weight','update')
        self.updated_attribute('mesh_id','update')