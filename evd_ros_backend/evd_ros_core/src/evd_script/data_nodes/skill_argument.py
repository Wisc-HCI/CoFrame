'''
Skill argument provides a simple typing scheme for parameters to be passed
into skills when invoked. Effectively, each value in the underlying nodes
are overrided with a temporary value and copied over to a real value on
invocation.
'''

from .. import ALL_NODES_TYPE, BOOLEAN_TYPE, STRING_TYPE
from ..node import Node


class SkillArgument(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Skill Argument'

    @classmethod
    def type_string(cls, trailing_delim):
        return 'skill-argument' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Node.template()
        template['fields'].append({
            'type': STRING_TYPE,
            'key': 'parameter_key',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': STRING_TYPE,
            'key': 'temporary_value',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': STRING_TYPE,
            'key': 'parameter_type',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': BOOLEAN_TYPE,
            'key': 'is_list',
            'is_uuid': False,
            'is_list': False
        })
        return template
    
    def __init__(self, parameter_key=None, temporary_value=None, parameter_type=ALL_NODES_TYPE, 
                 is_list=False, type='', name='', uuid=None, parent=None, append_type=True, 
                 editable=True, deleteable=True, description=''):

        super(SkillArgument,self).__init__(
            type=SkillArgument.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        if parameter_key == None:
            raise Exception('parameter_key must be defined at construction')

        self._parameter_key = parameter_key
        self._temporary_value = temporary_value if temporary_value != None else self._generate_uuid('skill-argument-variable')
        self._parameter_type = parameter_type
        self._is_list = is_list

        self.updated_attribute('parameter_key','set')
        self.updated_attribute('temporary_value','set')
        self.updated_attribute('parameter_type','set')
        self.updated_attribute('is_list','set')

    def to_dct(self):
        msg = super(SkillArgument,self).to_dct()
        msg.update({
            'parameter_key': self.parameter_key,
            'temporary_value': self.temporary_value,
            'parameter_type': self.parameter_type,
            'is_list': self.is_list
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            type=dct['type'],
            append_type=False,
            editable=dct['editable'],
            deleteable=dct['deleteable'],
            description=dct['description'],
            uuid=dct['uuid'],
            parameter_key=dct['parameter_key'],
            temporary_value=dct['temporary_value'],
            parameter_type=dct['parameter_type'],
            is_list=dct['is_list'])

    '''
    Data accessor/modifier methods
    '''

    @property
    def parameter_key(self):
        return self._parameter_key

    @property
    def temporary_value(self):
        return self._temporary_value

    @property
    def parameter_type(self):
        return self._parameter_type

    @property
    def is_list(self):
        return self._is_list

    '''
    Update methods
    '''

    def deep_update(self):
        super(SkillArgument,self).deep_update()

        self.updated_attribute('parameter_key','update')
        self.updated_attribute('temporary_value','update')
        self.updated_attribute('parameter_type','update')
        self.updated_attribute('is_list','update')

    def shallow_update(self):
        super(SkillArgument,self).shallow_update()

        self.updated_attribute('parameter_key','update')
        self.updated_attribute('temporary_value','update')
        self.updated_attribute('parameter_type','update')
        self.updated_attribute('is_list','update')