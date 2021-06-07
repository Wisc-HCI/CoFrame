'''
Primitive is a sub-type of node that indicates it can be executed in a program.
'''

from ..node import Node
from .. import PARAMETERS_FIELD_DCT

class Primitive(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Primitive'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'primitive' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Node.template()
        template['fields'].append({
            'type': PARAMETERS_FIELD_DCT,
            'key': 'parameters',
            'is_uuid': False,
            'is_list': False
        })
        template['parameters'] = [] # define a space to spec out parameters of executable nodes

    def __init__(self, parameters=None, type='', name='', uuid=None, parent=None, append_type=True,
                 editable=True, deleteable=True, description=''):
        self._parameters = None

        super(Primitive,self).__init__(
            type=Primitive.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        self.parameters = parameters if parameters != None else {}

    def to_dct(self):
        msg = super(Primitive,self).to_dct()
        msg.update({
            'parameters': self.parameters
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            uuid=dct['uuid'],
            type=dct['type'],
            append_type=False,
            editable=dct['editable'],
            deleteable=dct['deleteable'],
            description=dct['description'],
            parameters=dct['parameters'])

    '''
    Data accessor/modifier methods
    '''

    @property
    def parameters(self):
        return self._parameters

    @parameters.setter
    def parameters(self, value):
        if value != self._parameters:
            if not isinstance(value,dict):
                raise Exception('Parameters must be a valid dictionary')

            self._parameters = value
            self.updated_attribute('parameters','set')

            for param in self._parameters.keys():
                self.updated_attribute('parameters.{}'.format(param),'set')

    def set(self, dct):

        if 'parameters' in dct.keys():
            self.parameters = dct['parameters']

        super(Primitive,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):
        super(Primitive,self).deep_update()

        self.updated_attribute('parameters','update')

        for param in self._parameters.keys():
                self.updated_attribute('parameters.{}'.format(param),'update')

    def shallow_update(self):
        super(Primitive,self).shallow_update()

        self.updated_attribute('parameters','update')

        for param in self._parameters.keys():
                self.updated_attribute('parameters.{}'.format(param),'update')