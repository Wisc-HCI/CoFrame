'''
Things are objects being manipulated by the robot and/or produced consumed by machines.

These act as tokens within the simulator and may be useful for verification in the future.

The thinking behind things is much the same as it was in Authr.
'''

from .thing_type import ThingType
from ..node_parser import NodeParser
from .geometry import Pose, Position, Orientation
from ..visualizable import VisualizeMarker, ColorTable

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3


class Thing(Pose, VisualizeMarker):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Thing'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'thing' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Pose.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Pose.template()
        template['fields'].append({
            'type': ThingType.full_type_string(),
            'key': 'thing_type_uuid',
            'is_uuid': True,
            'is_list': False
        })
        return template

    def __init__(self, thing_type_uuid, position=None, orientation=None, link=None,
                 type='', name='', parent=None, uuid=None, append_type=True,
                 editable=True, deleteable=True, description=''):
        self._thing_type_uuid = None

        super(Thing,self).__init__(
            position=position,
            orientation=orientation,
            link='app' if link == None else link,
            type=Thing.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        self.thing_type_uuid = thing_type_uuid

    def to_dct(self):
        msg = super(Thing,self).to_dct()
        msg.update({
            'thing_type_uuid': self.thing_type_uuid
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(thing_type_uuid=dct['thing_type_uuid'],
                   position=NodeParser(dct['position'], enforce_types=[Position.type_string(trailing_delim=False)]),
                   orientation=NodeParser(dct['orientation'], enforce_types=[Orientation.type_string(trailing_delim=False)]),
                   link=dct['link'],
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   editable=dct['editable'],
                   deleteable=dct['deleteable'],
                   description=dct['description'],
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    def to_ros_marker(self, frame_id=None, id=0):
        # The frame_id should be the application frame
        if frame_id == None:
            frame_id = self.link if self.link != "" and self.link != None else "app"

        marker = None

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.MESH_RESOURCE
        marker.ns = 'things'
        marker.id = id
        marker.pose = self.to_ros()
        marker.scale = Vector3(1,1,1)
        marker.color = ColorTable.THING_COLOR
        marker.mesh_resource = self.context.get_thing_type(self.thing_type_uuid).mesh_id

        return marker

    '''
    Data accessor/modifier methods
    '''

    @property
    def thing_type_uuid(self):
        return self._thing_type_uuid

    @thing_type_uuid.setter
    def thing_type_uuid(self, value):
        if self._thing_type_uuid != value:
            if value == None:
                raise Exception('Thing type must exist')

            self._thing_type_uuid = value
            self.updated_attribute('thing_type_uuid','set')

    def set(self, dct):

        if 'thing_type_uuid' in dct.keys():
            self.thing_type = dct['thing_type_uuid']

        super(Thing,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):

        super(Thing,self).deep_update()

        self.updated_attribute('thing_type_uuid','update')
        self.updated_attribute('mesh_id','update')

    def shallow_update(self):
        super(Thing,self).shallow_update()

        self.updated_attribute('thing_type_uuid','update')
        self.updated_attribute('mesh_id','update')
