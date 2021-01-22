from ..node import Node


class MachineRecipe(Node):

    '''
    Data structure methods
    '''

    def __init__(self, process_time=0, input_thing_quantities={}, output_thing_quantities={},
                 type='', name='', uuid=None, parent=None, append_type=True):
        self._process_time = None
        self._input_quantities = None
        self._output_quantities = None

        super(MachineRecipe,self).__init__(
            type='machine-recipe.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

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

    def add_input_thing_quantity(self, thing_type, quantity, override=False):
        verb = 'add'

        if thing_type in self._input_quantities.keys():
            if not override:
                raise Exception('Thing Quantity is about to be overrided, override is not allowed')
            else:
                verb = 'update'

        self._input_quantities[thing_type] = quantity
        self.updated_attribute('input_thing_quantities',verb)

    def delete_input_thing_quantity(self, thing_type):
        if not thing_type in self._input_quantities.keys():
            raise Exception('No such thing `{0}` in input_quantities'.format(thing_type))

        obj = self._input_quantities.pop(thing_type)
        self.updated_attribute('input_thing_quantities','delete')

    def get_input_thing_quantity(self, thing_type):
        return self._input_quantities[thing_type]

    @property
    def output_thing_quantities(self):
        return self._output_quantities

    @output_thing_quantities.setter
    def output_thing_quantities(self, value):
        if self._output_quantities != value:
            self._output_quantities = value
            self.updated_attribute('output_thing_quantities','set')

    def add_output_thing_quantity(self, thing_type, quantity, override=False):
        verb = 'add'

        if thing_type in self._output_quantities.keys():
            if not override:
                raise Exception('Thing Quantity is about to be overrided, override is not allowed')
            else:
                verb = 'update'

        self._output_quantities[thing_type] = quantity
        self.updated_attribute('output_thing_quantities',verb)

    def delete_output_thing_quantity(self, thing_type):
        if not thing_type in self._output_quantities.keys():
            raise Exception('No such thing `{0}` in output_quantities'.format(thing_type))

        obj = self._output_quantities.pop(thing_type)
        self.updated_attribute('output_thing_quantities','delete')

    def get_output_thing_quantity(self, thing_type):
        return self._output_quantities[thing_type]

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


class Machine(Node):

    '''
    Data structure methods
    '''

    def __init__(self, collision_mesh_uuid=None, input_regions=None, output_regions=None,
                 recipe=None, type='', name='', uuid=None, parent=None, append_type=True):
        self._collision_mesh_uuid = None
        self._input_regions = None
        self._output_regions = None
        self._machine_type = None
        self._recipe = None

        super(Machine,self).__init__(
            type='machine.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.collision_mesh_uuid = collision_mesh_uuid
        self.input_regions = input_regions
        self.output_regions = output_regions
        self.recipe = recipe if recipe != None else MachineRecipe()

    def to_dct(self):
        msg = super(Machine,self).to_dct()
        msg.update({
            'collision_mesh_uuid': self.collision_mesh_uuid,
            'input_regions': {k: self.input_regions[k].to_dct() for k in self.input_regions.keys()} if self.input_regions != None else None,
            'output_regions': {k: self.output_regions[k].to_dct() for k in self.output_regions.keys()} if self.output_regions != None else None,
            'recipe': self.recipe.to_dct()
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        from ..utility_functions import NodeParser

        return cls(
            collision_mesh_uuid=dct['collision_mesh_uuid'],
            input_regions={k: NodeParser(dct[k]) for k in dct['input_regions'].keys()} if dct['input_regions'] != None else None,
            output_regions={k: NodeParser(dct[k]) for k in dct['output_regions'].keys()} if dct['output_regions'] != None else None,
            recipe=MachineRecipe.from_dct(dct['recipe']),
            type=dct['type'] if 'type' in dct.keys() else '',
            append_type=not 'type' in dct.keys(),
            uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
            name=dct['name'] if 'name' in dct.keys() else '')

    '''
    Data accessor/modifier methods
    '''

    @property
    def collision_mesh_uuid(self):
        return self._collision_mesh_uuid

    @collision_mesh_uuid.setter
    def collision_mesh_uuid(self, value):
        if self._collision_mesh_uuid != value:
            if self._collision_mesh_uuid != None:
                self._collision_mesh_uuid.remove_from_cache()

            self._collision_mesh_uuid = value
            if self._collision_mesh_uuid != None:
                self._collision_mesh_uuid.parent = self

            self.updated_attribute('collision_mesh_uuid','set')

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

            self.updated_attribute('input_regions','set')

    def add_input_region(self, thing_type, region, override=False):
        verb = 'add'

        if thing_type in self._input_regions.keys():
            if not override:
                raise Exception('Thing Region is about to be overrided, override is not allowed')
            else:
                self._input_regions[thing_type].remove_from_cache()
                verb = 'update'

        region.parent = self
        self._input_regions[thing_type] = region
        self.updated_attribute('input_regions',verb,region.uuid)

    def delete_input_region(self, thing_type):
        if not thing_type in self._input_regions.keys():
            raise Exception('No such thing `{0}` in input_regions'.format(thing_type))

        self._input_regions[thing_type].remove_from_cache()
        obj = self._input_regions.pop(thing_type)
        self.updated_attribute('input_regions','delete',obj.uuid)

    def get_input_region(self, thing_type):
        return self._input_regions[thing_type]

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

            self.updated_attribute('output_regions','set')

    def add_output_region(self, thing_type, region):
        verb = 'add'

        if thing_type in self._output_regions.keys():
            if not override:
                raise Exception('Thing Region is about to be overrided, override is not allowed')
            else:
                self._output_regions[thing_type].remove_from_cache()
                verb = 'update'

        region.parent = self
        self._output_regions[thing_type] = region
        self.updated_attribute('output_regions',verb,region.uuid)

    def delete_output_region(self, thing_type):
        if not thing_type in self._output_regions.keys():
            raise Exception('No such thing `{0}` in output_regions'.format(thing_type))

        self._output_regions[thing_type].remove_from_cache()
        obj = self._output_regions.pop(thing_type)
        self.updated_attribute('output_regions','delete',obj.uuid)

    def get_output_region(self, thing_type):
        return self._output_regions[thing_type]

    def find_input_region_thing_type(self, regionId):
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
        from ..utility_functions import NodeParser

        if 'collision_mesh_uuid' in dct.keys():
            self.collision_mesh_uuid = dct['collision_mesh_uuid']

        if 'input_regions' in dct.keys():
            self.input_regions = {k: NodeParser(dct[k]) for k in dct['input_regions'].keys()}

        if 'output_regions' in dct.keys():
            self.output_regions = {k: NodeParser(dct[k]) for k in dct['output_regions'].keys()}

        if 'recipe' in dct.keys():
            self.recipe = MachineRecipe.from_dct(dct['recipe'])

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

    def deep_update(self):
        super(Machine,self).deep_update()

        self.updated_attribute('machine_type','update')
        self.updated_attribute('collision_mesh_uuid','update')
        self.updated_attribute('input_regions','update')
        self.updated_attribute('output_regions','update')
        self.updated_attribute('recipe','update')

    def shallow_update(self):
        super(Machine,self).shallow_update()

        self.updated_attribute('machine_type','update')
        self.updated_attribute('collision_mesh_uuid','update')
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
