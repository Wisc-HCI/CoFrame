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
from ..node_parser import NodeParser
from .machine_recipe import MachineRecipe


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

    def __init__(self, input_regions=None, output_regions=None,
                 recipe=None, type='', name='', uuid=None, parent=None, append_type=True, editable=True, deleteable=True):
        self._input_regions = None
        self._output_regions = None
        self._machine_type = None
        self._recipe = None

        super(Machine,self).__init__(
            type=Machine.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable)

        self.input_regions = input_regions
        self.output_regions = output_regions
        self.recipe = recipe if recipe != None else MachineRecipe()

    def to_dct(self):
        msg = super(Machine,self).to_dct()
        msg.update({
            'input_regions': {k: self.input_regions[k].to_dct() for k in self.input_regions.keys()} if self.input_regions != None else None,
            'output_regions': {k: self.output_regions[k].to_dct() for k in self.output_regions.keys()} if self.output_regions != None else None,
            'recipe': self.recipe.to_dct()
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            input_regions={k: NodeParser(dct[k]) for k in dct['input_regions'].keys()} if dct['input_regions'] != None else None,
            output_regions={k: NodeParser(dct[k]) for k in dct['output_regions'].keys()} if dct['output_regions'] != None else None,
            recipe=NodeParser(dct['recipe'], enforce_type=MachineRecipe.type_string(trailing_delim=False)),
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
    def input_regions(self):
        return self._input_regions

    @input_regions.setter
    def input_regions(self, value):
        if self._input_regions != value:
            for k in self._input_regions.keys():
                self._input_regions[k].remove_from_cache()

            self._input_regions = value
            for k in self._input_regions.keys():
                self._input_regions[k].parent = self

            self._compute_type()
            self.updated_attribute('input_regions','set')

    def add_input_region(self, thing_type_uuid, region, override=False):
        verb = 'add'

        if thing_type_uuid in self._input_regions.keys():
            if not override:
                raise Exception('Thing Region is about to be overrided, override is not allowed')
            else:
                self._input_regions[thing_type_uuid].remove_from_cache()
                verb = 'update'

        region.parent = self
        self._input_regions[thing_type_uuid] = region
        self._compute_type()
        self.updated_attribute('input_regions',verb,region.uuid)

    def delete_input_region(self, thing_type_uuid):
        if not thing_type_uuid in self._input_regions.keys():
            raise Exception('No such thing `{0}` in input_regions'.format(thing_type_uuid))

        self._input_regions[thing_type_uuid].remove_from_cache()
        obj = self._input_regions.pop(thing_type_uuid)
        self._compute_type()
        self.updated_attribute('input_regions','delete',obj.uuid)

    def get_input_region(self, thing_type_uuid):
        return self._input_regions[thing_type_uuid]

    def find_input_region_thing_type(self, regionId):
        type = None

        for k in self._input_regions.keys():
            if self._input_regions[k].uuid == regionId:
                type = k
                break

        return type

    @property
    def output_regions(self):
        return self._output_regions

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

    def add_output_region(self, thing_type_uuid, region, override=False):
        verb = 'add'

        if thing_type_uuid in self._output_regions.keys():
            if not override:
                raise Exception('Thing Region is about to be overrided, override is not allowed')
            else:
                self._output_regions[thing_type_uuid].remove_from_cache()
                verb = 'update'

        region.parent = self
        self._output_regions[thing_type_uuid] = region
        self._compute_type()
        self.updated_attribute('output_regions',verb,region.uuid)

    def delete_output_region(self, thing_type_uuid):
        if not thing_type_uuid in self._output_regions.keys():
            raise Exception('No such thing `{0}` in output_regions'.format(thing_type_uuid))

        self._output_regions[thing_type_uuid].remove_from_cache()
        obj = self._output_regions.pop(thing_type_uuid)
        self._compute_type()
        self.updated_attribute('output_regions','delete',obj.uuid)

    def get_output_region(self, thing_type_uuid):
        return self._output_regions[thing_type_uuid]

    def find_output_region_thing_type(self, regionId):
        type = None

        for k in self._output_regions.keys():
            if self._output_regions[k].uuid == regionId:
                type = k
                break

        return type

    @property
    def recipe(self):
        return self._recipe

    @recipe.setter
    def recipe(self, value):
        if self._recipe != value:
            if value == None:
                raise Exception('A recipe can be empty but it must exist!')

            if self._recipe != None:
                self._recipe.remove_from_cache()

            self._recipe = value
            self._recipe.parent = self
            self.updated_attribute('recipe','set')

    def set(self, dct):

        if 'input_regions' in dct.keys():
            self.input_regions = {k: NodeParser(dct[k]) for k in dct['input_regions'].keys()}

        if 'output_regions' in dct.keys():
            self.output_regions = {k: NodeParser(dct[k]) for k in dct['output_regions'].keys()}

        if 'recipe' in dct.keys():
            self.recipe = NodeParser(dct['recipe'], enforce_type=MachineRecipe.type_string(trailing_delim=False))

        super(Machine,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):

        if self.input_regions != None:
            for k in self.input_regions.keys():
                self.input_regions[k].remove_from_cache()

        if self.output_regions != None:
            for k in self.output_regions.keys():
                self.output_regions[k].remove_from_cache()

        self.recipe.remove_from_cache()

        super(Machine,self).remove_from_cache()

    def add_to_cache(self):

        if self.input_regions != None:
            for k in self.input_regions.keys():
                self.input_regions[k].add_to_cache()

        if self.output_regions != None:
            for k in self.output_regions.keys():
                self.output_regions[k].add_to_cache()

        self.recipe.add_to_cache()

        super(Machine,self).add_to_cache()

    '''
    Update Methods
    '''

    def late_construct_update(self):

        if self.input_regions != None:
            for i in self.input_regions.values():
                i.late_construct_update()

        if self.output_regions != None:
            for o in self.output_regions.values():
                o.late_construct_update()

        self.recipe.late_construct_update()

        super(Machine,self).late_construct_update()

    def deep_update(self):

        if self.input_regions != None:
            for i in self.input_regions.values():
                i.deep_update()

        if self.output_regions != None:
            for o in self.output_regions.values():
                o.deep_update()

        self.recipe.deep_update()

        super(Machine,self).deep_update()

        self.updated_attribute('machine_type','update')
        self.updated_attribute('input_regions','update')
        self.updated_attribute('output_regions','update')
        self.updated_attribute('recipe','update')

    def shallow_update(self):
        super(Machine,self).shallow_update()

        self.updated_attribute('machine_type','update')
        self.updated_attribute('input_regions','update')
        self.updated_attribute('output_regions','update')
        self.updated_attribute('recipe','update')

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
