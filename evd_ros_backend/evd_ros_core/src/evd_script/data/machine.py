'''
A machine is an "agent" that is neither human nor the robot executing the program.
Each machine can generate and/or consume things.

Machine could be something "smart" like a CNC mill or a 3D printer, but it could also be
a simple machine like a feeder. Additionally, vision systems can be a machine that
"generates" things by finding them in the real environment.

A machine allows for a set of input regions that take in a thing of a particular type and a set of output
regions that generate things of a certain type. Machines also use a recipe to define the number of input
things at the input regions that can be converted into a number of output things at the output regions.

Machines can be purely generators (they don't have any inputs), purely consumers (no outputs), or
transformers (that have inputs and outputs).

NOTE, currently regions are not a supported type for robot planning. Robot's use locations so arbitrary
position is not supported. This is fine for the studies at this time but more complex use will require
handling this disconnect.
'''

from ..node import Node


class Machine(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'machine' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    def __init__(self, inputs=None, outputs=None, process_time=0, type='', name='',
                 uuid=None, parent=None, append_type=True, editable=True,
                 deleteable=True, description=description):
        self._inputs = None
        self._outputs = None
        self._process_time = None
        self._machine_type = None

        super(Machine,self).__init__(
            type=Machine.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        self.inputs = inputs if inputs != None else {}
        self.outputs = outputs if outputs != None else {}
        self.process_time = process_time

    def to_dct(self):
        msg = super(Machine,self).to_dct()
        msg.update({
            'inputs': self.inputs,
            'outputs': self.outputs,
            'process_time': self.process_time
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            inputs=dct['inputs'],
            outputs=dct['outputs'],
            process_time=dct['process_time'],
            type=dct['type'] if 'type' in dct.keys() else '',
            append_type=not 'type' in dct.keys(),
            uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
            name=dct['name'] if 'name' in dct.keys() else '')

    '''
    Data accessor/modifier methods
    '''

    @property
    def machine_type(self):
        return self._machine_type

    @property
    def inputs(self):
        return self._inputs

    @inputs.setter
    def inputs(self, value):
        if self._inputs != value:
            if value == None:
                raise Exception('Must be a valid dictionary')

            self._inputs = value

            self._compute_type()
            self.updated_attribute('inputs','set')

    def add_input_region(self, thing_type_uuid, region_uuid, quantity, override=False):
        verb = 'add'

        if not thing_type_uuid in self._inputs.keys():
            self._inputs[thing_type_uuid] = []

        found = False
        for i in range(0,len(self._inputs[thing_type_uuid])):
            if region_uuid in self._inputs[thing_type_uuid][i]['region_uuid']:
                if not override:
                    raise Exception('Region already exists, cannot add')
                else:
                    self._inputs[i]['quantity'] = quantity
                    verb = 'set'
                found = True
                break

        if not found:
            self._inputs[thing_type_uuid].append({
                'region_uuid': region_uuid,
                'quantity': quantity })

        self._compute_type()
        self.updated_attribute('inputs',verb,region_uuid)

    def delete_input_region(self, thing_type_uuid, region_uuid):
        if not thing_type_uuid in self._inputs.keys():
            raise Exception('No such thing `{0}` in inputs'.format(thing_type_uuid))

        idx = None
        for i in range(0,len(self._inputs[thing_type_uuid])):
            if region_uuid in self._inputs[thing_type_uuid][i]['region_uuid']:
                idx = i
                break

        if idx == None:
            raise Exception('Region `{0}` not in inputs'.format(region_uuid))
        else:
            self._inputs[thing_type_uuid].pop(idx)
            self.updated_attribute('inputs','delete',region_uuid)

            if len(self._inputs[thing_type_uuid]) == 0:
                del self._inputs[thing_type_uuid]
                self.updated_attribute('inputs','delete',thing_type_uuid)

        self._compute_type()

    def set_input_region_quantity(self, thing_type_uuid, region_uuid, quantity):
        if not thing_type_uuid in self._inputs.keys():
            raise Exception('No such thing `{0}` in inputs'.format(thing_type_uuid))

        idx = None
        for i in range(0,len(self._inputs[thing_type_uuid])):
            if region_uuid in self._inputs[thing_type_uuid][i]['region_uuid']:
                idx = i
                break

        if idx == None:
            raise Exception('Region `{0}` not in inputs'.format(region_uuid))
        else:
            self._inputs[thing_type_uuid][idx]['quantity'] = quantity
            self.updated_attribute('inputs','set',region_uuid)

        self._compute_type()
        self.updated_attribute('inputs',verb,region_uuid)

    @property
    def outputs(self):
        return self._outputs

    @output_regions.setter
    def output_regions(self, value):
        if self._output_regions != value:
            for k in self._output_regions.keys():
                self._output_regions[k].remove_from_cache()

            self._output_regions = value
            for k in self._output_regions.keys():
                self._output_regions[k].parent = self

            self._compute_type()
            self.updated_attribute('output_regions','set')

    def add_output_region(self, thing_type_uuid, region_uuid, override=False):
        verb = 'add'

        if not thing_type_uuid in self._outputs.keys():
            self._outputs[thing_type_uuid] = []

        found = False
        for i in range(0,len(self._outputs[thing_type_uuid])):
            if region_uuid in self._outputs[thing_type_uuid][i]['region_uuid']:
                if not override:
                    raise Exception('Region already exists, cannot add')
                else:
                    self._outputs[i]['quantity'] = quantity
                    verb = 'set'
                found = True
                break

        if not found:
            self._outputs[thing_type_uuid].append({
                'region_uuid': region_uuid,
                'quantity': quantity })

        self._compute_type()
        self.updated_attribute('inputs',verb,region_uuid)

    def delete_output_region(self, thing_type_uuid, region_uuid):
        if not thing_type_uuid in self._outputs.keys():
            raise Exception('No such thing `{0}` in _outputs'.format(thing_type_uuid))

        idx = None
        for i in range(0,len(self._outputs[thing_type_uuid])):
            if region_uuid in self._outputs[thing_type_uuid][i]['region_uuid']:
                idx = i
                break

        if idx == None:
            raise Exception('Region `{0}` not in outputs'.format(region_uuid))
        else:
            self._outputs[thing_type_uuid].pop(idx)
            self.updated_attribute('outputs','delete',region_uuid)

            if len(self._outputs[thing_type_uuid]) == 0:
                del self._outputs[thing_type_uuid]
                self.updated_attribute('outputs','delete',thing_type_uuid)

        self._compute_type()

    def set_output_region_quantity(self, thing_type_uuid, region_uuid, quantity):
        if not thing_type_uuid in self._outputs.keys():
            raise Exception('No such thing `{0}` in outputs'.format(thing_type_uuid))

        idx = None
        for i in range(0,len(self._outputs[thing_type_uuid])):
            if region_uuid in self._outputs[thing_type_uuid][i]['region_uuid']:
                idx = i
                break

        if idx == None:
            raise Exception('Region `{0}` not in outputs'.format(region_uuid))
        else:
            self._outputs[thing_type_uuid][idx]['quantity'] = quantity
            self.updated_attribute('outputs','set',region_uuid)

        self._compute_type()
        self.updated_attribute('outputs',verb,region_uuid)

    @property
    def process_time(self):
        return self._process_time

    @process_time.setter
    def process_time(self, value):
        if self._process_time != value:
            self._process_time = value
            self.updated_attribute('process_time','set')


    def set(self, dct):

        if 'inputs' in dct.keys():
            self.inputs = dct['inputs']

        if 'outputs' in dct.keys():
            self.outputs = dct['outputs']

        if 'process_time' in dct.keys():
            self.process_time = dct['process_time']

        super(Machine,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):
        super(Machine,self).deep_update()

        self.updated_attribute('machine_type','update')
        self.updated_attribute('inputs','update')
        self.updated_attribute('outputs','update')
        self.updated_attribute('process_time','update')

    def shallow_update(self):
        super(Machine,self).shallow_update()

        self.updated_attribute('machine_type','update')
        self.updated_attribute('inputs','update')
        self.updated_attribute('outputs','update')
        self.updated_attribute('process_time','update')

    '''
    Utility Methods
    '''

    def _compute_type(self):
        type = None

        if self.input_regions == None and self.output_regions == None:
            type = 'useless'
        elif self.input_regions == None and self.output_regions != None:
            if len(self.output_regions) > 0:
                type = 'generator'
            else:
                type = 'useless'
        elif self.input_regions != None and self.output_regions == None:
            if len(self.input_regions) > 0:
                type = 'consumer'
            else:
                type = 'useless'
        else:
            type = 'transformer'

        self._machine_type = type
        self.updated_attribute('machine_type','set')
