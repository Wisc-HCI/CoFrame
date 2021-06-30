'''
Placeholder acts as substitute for other node in EvD that spawn from regions. If you 
are familiar with Promises from javascript, this is similar.

When machines operate, they produce things according to a thing_type. Until they do, there
is no actual thing. However, in order to program evd we need to explicitly represent a
node that acts as a thing beforehand.

Likewise, it would be nice to have variable locations that get generated by a region. This
way we can do palletization things.

Both of these can be accomplished by promising a type (templated dict to convert into a node)
and a set of fields subject to change. 
'''

from ..node import Node
from ..node_parser import NodeParser
from ..type_defs import ARBITRARY_OBJ_TYPE, STRING_TYPE


class Placeholder(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Placeholder'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'placeholder' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Node.template()
        template['fields'].append({
            'type': ARBITRARY_OBJ_TYPE,
            'key': 'pending_node',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': STRING_TYPE,
            'key': 'pending_fields',
            'is_uuid': False,
            'is_list': False
        })
        return template

    def __init__(self, pending_node_dct=None, pending_fields=[], type='', name='', 
                 uuid=None, parent=None, append_type=True, editable=True, 
                 deleteable=True, description=''):

        super(Placeholder,self).__init__(
            type=Placeholder.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        self._pending_node_dct = pending_node_dct
        self._pending_fields = pending_fields
        self._pending_type = pending_node_dct['type']

        for field in self._pending_fields:
            self._pending_node_dct[field] = '<pending>'

        self.updated_attribute('pending_node_dct','set')
        self.updated_attribute('pending_fields','set')
        self.updated_attribute('pending_type','set')

    def to_dct(self):
        msg = super(Placeholder,self).to_dct()
        msg.update({
            'pending_node_dct': self._pending_node_dct,
            'pending_fields': self._pending_fields
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            type=dct['type'],
            append_type=False,
            editable=dct['editable'],
            deleteble=dct['deleteable'],
            description=dct['description'],
            uuid=dct['uuid'],
            pending_node_dct=dct['pending_node_dct'],
            pending_fields=dct['pending_fields'])

    '''
    Data accessor/modifier methods
    '''

    @property
    def pending_node_dct(self):
        return self._pending_node_dct

    @property
    def pending_fields(self):
        return self._pending_fields

    '''
    Update methods
    '''

    def deep_update(self):
        super(Placeholder,self).deep_update()

        self.updated_attribute('pending_node_dct','update')
        self.updated_attribute('pending_fields','update')
        self.updated_attribute('pending_type','update')

    def shallow_update(self):
        super(Placeholder,self).shallow_update()

        self.updated_attribute('pending_node_dct','update')
        self.updated_attribute('pending_fields','update')
        self.updated_attribute('pending_type','update')

    '''
    Utility methods
    '''

    def resolve_placeholder(self, field_map):

        node = dict(self._pending_node_dct)

        for field in self._pending_fields:
            node[field] = field_map[field]

        return NodeParser(node)