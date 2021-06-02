'''
Machine recipe defines the number of things inputed to produce a number of things
outputed during a machining process. The process should also provide a static 
time estimate.
'''

from ..node import Node


class MachineRecipe(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'machine-recipe' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    def __init__(self, process_time=0, input_thing_quantities={}, output_thing_quantities={},
                 type='', name='', uuid=None, parent=None, append_type=True, editable=True, deleteable=True):
        self._process_time = None
        self._input_quantities = None
        self._output_quantities = None

        super(MachineRecipe,self).__init__(
            type=MachineRecipe.type_string + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable)

        self.process_time = process_time
        self.input_thing_quantities = input_thing_quantities
        self.output_thing_quantities = output_thing_quantities

    def to_dct(self):
        msg = super(MachineRecipe,self).to_dct()
        msg.update({
            'process_time': self.process_time,
            'input_thing_quantities': self.input_thing_quantities,
            'output_thing_quantities': self.output_thing_quantities
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            process_time=dct['process_time'],
            input_thing_quantities=dct['input_thing_quantities'],
            output_thing_quantities=dct['output_thing_quantities'],
            type=dct['type'] if 'type' in dct.keys() else '',
            append_type=not 'type' in dct.keys(),
            uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
            name=dct['name'] if 'name' in dct.keys() else '')

    '''
    Data accessor/modifier methods
    '''

    @property
    def process_time(self):
        return self._process_time

    @process_time.setter
    def process_time(self, value):
        if self._process_time != value:
            self._process_time = value
            self.updated_attribute('process_time','set')

    @property
    def input_thing_quantities(self):
        return self._input_quantities

    @input_thing_quantities.setter
    def input_thing_quantities(self, value):
        if self._input_quantities != value:
            self._input_quantities = value
            self.updated_attribute('input_thing_quantities','set')

    def add_input_thing_quantity(self, thing_type_uuid, quantity, override=False):
        verb = 'add'

        if thing_type_uuid in self._input_quantities.keys():
            if not override:
                raise Exception('Thing Quantity is about to be overrided, override is not allowed')
            else:
                verb = 'update'

        self._input_quantities[thing_type_uuid] = quantity
        self.updated_attribute('input_thing_quantities',verb)

    def delete_input_thing_quantity(self, thing_type_uuid):
        if not thing_type_uuid in self._input_quantities.keys():
            raise Exception('No such thing `{0}` in input_quantities'.format(thing_type_uuid))

        obj = self._input_quantities.pop(thing_type_uuid)
        self.updated_attribute('input_thing_quantities','delete')

    def get_input_thing_quantity(self, thing_type_uuid):
        return self._input_quantities[thing_type_uuid]

    @property
    def output_thing_quantities(self):
        return self._output_quantities

    @output_thing_quantities.setter
    def output_thing_quantities(self, value):
        if self._output_quantities != value:
            self._output_quantities = value
            self.updated_attribute('output_thing_quantities','set')

    def add_output_thing_quantity(self, thing_type_uuid, quantity, override=False):
        verb = 'add'

        if thing_type_uuid in self._output_quantities.keys():
            if not override:
                raise Exception('Thing Quantity is about to be overrided, override is not allowed')
            else:
                verb = 'update'

        self._output_quantities[thing_type_uuid] = quantity
        self.updated_attribute('output_thing_quantities',verb)

    def delete_output_thing_quantity(self, thing_type_uuid):
        if not thing_type_uuid in self._output_quantities.keys():
            raise Exception('No such thing `{0}` in output_quantities'.format(thing_type_uuid))

        obj = self._output_quantities.pop(thing_type_uuid)
        self.updated_attribute('output_thing_quantities','delete')

    def get_output_thing_quantity(self, thing_type_uuid):
        return self._output_quantities[thing_type_uuid]

    def set(self, dct):

        if 'process_time' in dct.keys():
            self.process_time = dct['process_time']

        if 'input_thing_quantities' in dct.keys():
            self.input_thing_quantities = dct['input_thing_quantities']

        if 'output_thing_quantities' in dct.keys():
            self.output_thing_quantities = dct['output_thing_quantities']

        super(MachineRecipe,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):
        super(MachineRecipe,self).deep_update()

        self.updated_attribute('process_time','update')
        self.updated_attribute('input_thing_quantities','update')
        self.updated_attribute('output_thing_quantities','update')

    def shallow_update(self):
        super(MachineRecipe,self).shallow_update()

        self.updated_attribute('process_time','update')
        self.updated_attribute('input_thing_quantities','update')
        self.updated_attribute('output_thing_quantities','update')
