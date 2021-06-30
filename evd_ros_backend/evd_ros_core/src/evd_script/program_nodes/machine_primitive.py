'''
Machine Primitive extends primitive to simplify the later definition of specific machine primitives.

A machine primitive is a way to generalize machine behavior in EvD. The actual behavior needs to be
supplied externally and is hooked into the EvD program runner.
'''

from .primitive import Primitive
from ..data_nodes.machine import Machine


class MachinePrimitive(Primitive):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Machine Primitive'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'machine-primitive' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Primitive.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Primitive.template()
        template['parameters'].append({
            'type': Machine.full_type_string(),
            'key': 'machine_uuid',
            'is_uuid': True,
            'is_list': False
        })
        return template

    def __init__(self, machine_uuid=None, parameters=None, type='', name='', uuid=None, parent=None,
                 append_type=True, editable=True, deleteable=True, description=''):

        if parameters == None:
            parameters = {
                'machine_uuid': machine_uuid
            }

        super(MachinePrimitive,self).__init__(
            type=MachinePrimitive.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description,
            parameters=parameters)

    '''
    Data accessor/modifier methods
    '''

    @property
    def machine_uuid(self):
        return self.parameters['machine_uuid']

    @machine_uuid.setter
    def machine_uuid(self, value):
        if self.parameters['machine_uuid'] != value:
            self.parameters['machine_uuid'] = value
            self.updated_attribute('parameters.machine_uuid','set')

    def set(self, dct):
        if 'machine_uuid' in dct.keys():
            self.machine_uuid = dct['machine_uuid']

        super(MachinePrimitive,self).set(dct)
