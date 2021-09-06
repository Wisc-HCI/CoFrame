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
handling this disconnect. Probably something like a variable location?
'''

from ..environment_nodes.collision_mesh import CollisionMesh
from ..type_defs import ARBITRARY_OBJ_TYPE, NUMBER_TYPE, STRING_TYPE, BOOLEAN_TYPE
from ..node import Node
from .geometry import Pose
from .placeholder import Placeholder
from ..node_parser import NodeParser


class Machine(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Machine'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'machine' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Node.template()
        template['fields'].append({
            'type': ARBITRARY_OBJ_TYPE, # dictionary of lists of dictionaries of regions and quantities with thing_type uuids as top-level keys
            'key': 'inputs',
            'is_uuid': False,
            'is_list': False 
        })
        template['fields'].append({
            'type': ARBITRARY_OBJ_TYPE, # dictionary of lists of dictionaries of regions and quantities with thing_type uuids as top-level keys
            'key': 'outputs',
            'is_uuid': False,
            'is_list': False 
        })
        template['fields'].append({
            'type': NUMBER_TYPE,
            'key': 'process_time',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': STRING_TYPE,
            'key': 'mesh_id',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': Pose.full_type_string(),
            'key': 'pose_offset',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': STRING_TYPE,
            'key': 'link',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': CollisionMesh.full_type_string(),
            'key': 'collision_mesh_uuid',
            'is_uuid': True,
            'is_list': False
        })
        template['fields'].append({
            'type': BOOLEAN_TYPE,
            'key': 'passive',
            'is_uuid': False,
            'is_list': False
        })
        return template

    def __init__(self, inputs=None, outputs=None, process_time=0, link='', mesh_id=None, 
                 passive=False, pose_offset=None, collision_mesh_uuid=None, type='', 
                 name='', uuid=None, parent=None, append_type=True, editable=True, 
                 deleteable=True, description=''):
        self._inputs = None
        self._outputs = None
        self._process_time = None
        self._machine_type = None
        self._mesh_id = None
        self._pose_offset = None
        self._link = None
        self._collision_mesh_uuid = None
        self._passive = None

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
        self.mesh_id = mesh_id
        self.pose_offset = pose_offset if pose_offset != None else Pose(link=link, deleteable=False, editable=editable)
        self.link = link
        self.collision_mesh_uuid = collision_mesh_uuid
        self.passive = passive

    def to_dct(self):
        msg = super(Machine,self).to_dct()
        msg.update({
            'inputs': self.inputs,
            'outputs': self.outputs,
            'process_time': self.process_time,
            'mesh_id': self.mesh_id,
            'pose_offset': self.pose_offset.to_dct(),
            'link': self.link,
            'collision_mesh_uuid': self.collision_mesh_uuid,
            'passive': self.passive
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            inputs=dct['inputs'],
            outputs=dct['outputs'],
            process_time=dct['process_time'],
            mesh_id=dct['mesh_id'],
            passive=dct['passive'],
            pose_offset=NodeParser(dct['pose_offset'], enforce_types=[Pose.type_string(trailing_delim=False)]),
            link=dct['link'],
            collision_mesh_uuid=dct['collision_mesh_uuid'],
            type=dct['type'] if 'type' in dct.keys() else '',
            append_type=not 'type' in dct.keys(),
            editable=dct['editable'],
            deleteable=dct['deleteable'],
            description=dct['description'],
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

        self._compute_type()
        self.updated_attribute('inputs','set',region_uuid)

    @property
    def outputs(self):
        return self._outputs

    @outputs.setter
    def outputs(self, value):
        if self._outputs != value:
            if value == None:
                raise Exception('Must be a valid dictionary')

            self._outputs = value

            self._compute_type()
            self.updated_attribute('outputs','set')

    def add_output_region(self, thing_type_uuid, region_uuid, placeholder_uuids, override=False):
        verb = 'add'

        if not thing_type_uuid in self._outputs.keys():
            self._outputs[thing_type_uuid] = []

        found = False
        for i in range(0,len(self._outputs[thing_type_uuid])):
            if region_uuid in self._outputs[thing_type_uuid][i]['region_uuid']:
                if not override:
                    raise Exception('Region already exists, cannot add')
                else:
                    self._outputs[i]['quantity'] = len(placeholder_uuids)
                    self._outputs[i]['placeholder_uuids'] = placeholder_uuids
                    verb = 'set'
                found = True
                break

        if not found:
            self._outputs[thing_type_uuid].append({
                'region_uuid': region_uuid,
                'quantity': len(placeholder_uuids),
                'placeholder_uuids': placeholder_uuids })

        self._compute_type()
        self.updated_attribute('outputs',verb,region_uuid)

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

    def change_output_placeholders(self, thing_type_uuid, region_uuid, placeholder_uuids):
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
            self._outputs[thing_type_uuid][idx]['quantity'] = len(placeholder_uuids)
            self._outputs[thing_type_uuid][idx]['placeholder_uuids'] = placeholder_uuids

        self._compute_type()
        self.updated_attribute('outputs','set',region_uuid)

    @property
    def process_time(self):
        return self._process_time

    @process_time.setter
    def process_time(self, value):
        if self._process_time != value:
            self._process_time = value
            self.updated_attribute('process_time','set')

    @property
    def mesh_id(self):
        return self._mesh_id

    @mesh_id.setter
    def mesh_id(self, value):
        if self._mesh_id != value:
            self._mesh_id = value
            self.updated_attribute('mesh_id','set')

    @property
    def passive(self):
        return self._passive

    @passive.setter
    def passive(self, value):
        if self._passive != value:
            self._passive = value
            self.updated_attribute('passive','set')

    @property
    def pose_offset(self):
        return self._pose_offset

    @pose_offset.setter
    def pose_offset(self, value):
        if self._pose_offset != value:
            if value == None:
                raise Exception('pose_offset cannot be None')

            if self._pose_offset != None:
                self._pose_offset.remove_from_cache()

            self._pose_offset = value
            if self._pose_offset != None:
                self._pose_offset.parent = self

            self.updated_attribute('pose_offset','set')

    @property
    def link(self):
        return self._link

    @link.setter
    def link(self, value):
        if self._link != value:
            self._link = value
            self.updated_attribute('link','set')

    @property
    def collision_mesh_uuid(self):
        return self._collision_mesh_uuid

    @collision_mesh_uuid.setter
    def collision_mesh_uuid(self, value):
        if self._collision_mesh_uuid != value:
            self._collision_mesh_uuid = value
            self.updated_attribute('collision_mesh_uuid','set')

    def set(self, dct):

        if 'inputs' in dct.keys():
            self.inputs = dct['inputs']

        if 'outputs' in dct.keys():
            self.outputs = dct['outputs']

        if 'process_time' in dct.keys():
            self.process_time = dct['process_time']

        if 'mesh_id' in dct.keys():
            self.mesh_id = dct['mesh_id']

        if 'pose_offset' in dct.keys():
            self.pose_offset = NodeParser(dct['pose_offset'], enforce_types=[Pose.type_string(trailing_delim=False)])

        if 'link' in dct.keys():
            self.link = dct['link']

        super(Machine,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):
        self.pose_offset.remove_from_cache()

        super(Machine,self).remove_from_cache()

    def add_to_cache(self):
        self.pose_offset.add_to_cache()

        super(Machine,self).add_to_cache()

    '''
    Update Methods
    '''

    def late_construct_update(self):
        self.pose_offset.late_construct_update()

        super(Machine,self).late_construct_update()

    def deep_update(self):
        super(Machine,self).deep_update()

        self.updated_attribute('machine_type','update')
        self.updated_attribute('inputs','update')
        self.updated_attribute('outputs','update')
        self.updated_attribute('process_time','update')
        self.updated_attribute('mesh_id','update')
        self.updated_attribute('pose_offset','update')
        self.updated_attribute('link','update')
        self.updated_attribute('collision_mesh_uuid','update')

    def shallow_update(self):
        super(Machine,self).shallow_update()

        self.updated_attribute('machine_type','update')
        self.updated_attribute('inputs','update')
        self.updated_attribute('outputs','update')
        self.updated_attribute('process_time','update')
        self.updated_attribute('mesh_id','update')
        self.updated_attribute('pose_offset','update')
        self.updated_attribute('link','update')
        self.updated_attribute('collision_mesh_uuid','update')

    '''
    Utility Methods
    '''

    def _compute_type(self):
        type = None

        if self.inputs == None and self.outputs == None:
            type = 'useless'
        elif self.inputs == None and self.outputs != None:
            if len(self.outputs) > 0:
                type = 'generator'
            else:
                type = 'useless'
        elif self.inputs != None and self.outputs == None:
            if len(self.inputs) > 0:
                type = 'consumer'
            else:
                type = 'useless'
        else:
            type = 'transformer'

        self._machine_type = type
        self.updated_attribute('machine_type','set')
