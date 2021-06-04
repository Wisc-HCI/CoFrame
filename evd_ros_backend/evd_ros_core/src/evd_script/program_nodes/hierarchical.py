'''
Hierarchical wraps a set of primitives that should be executed in order.

In hierarchical task analysis (and/or those familiar with Authr) this is a "task".
The purpose of this block is to abstract the base-primitives into more semantically
meaningful behaviors.
'''

from .primitive import Primitive
from ..node_parser import NodeParser


class Hierarchical(Primitive):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'hierarchical' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Primitive.full_type_string() + cls.type_string()

    def __init__(self, primitives=[], type='', name='', uuid=None, parent=None,
                 append_type=True, editable=True, deleteable=True, description=''):

        self._primitives = []

        super(Hierarchical,self).__init__(
            type=Hierarchical.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        self.primitives = primitives

    def to_dct(self):
        msg = super(Hierarchical,self).to_dct()
        msg.update({
            'primitives': [p.to_dct() for p in self.primitives]
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
            primitives=[NodeParser(p) for p in dct['primitives']])

    '''
    Data accessor/modifier methods
    '''

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, value):
        if self._parent != value:

            self.remove_from_cache()
            self._parent = value
            self.add_to_cache()

            self.updated_attribute("parent","set")

    @property
    def primitives(self):
        return self._primitives

    @primitives.setter
    def primitives(self, value):
        if self._primitives != value:
            for p in self._primitives:
                p.remove_from_cache()

            self._primitives = value
            for p in self._primitives:
                p.parent = self

            self.updated_attribute('primitives','set')

    def add_primitive(self, prm):
        prm.parent = self
        self._primitives.append(prm)
        self.updated_attribute('primitives','add')

    def insert_primitive(self, idx, prm):
        prm.parent = self
        self._primitives.insert(idx,prm)
        self.updated_attribute('primitives','reorder')

    def reorder_primitives(self, uuid, shift):
        idx = None
        for i in range(0,len(self._primitives)):
            if self._primitives[i].uuid == uuid:
                idx = i
                break

        if idx != None:
            shiftedIdx = idx + shift
            if shiftedIdx < 0 or shiftedIdx >= len(self._primitives):
                raise Exception("Index out of bounds")

            copy = self._primitives.pop(idx)
            self._primitives.insert(shiftedIdx,copy) #TODO check to make sure not off by one

            self.updated_attribute('primitives','reorder')

    def delete_primitive(self, uuid):
        delIdx = None
        for i in range(0,len(self._primitives)):
            if self._primitives[i].uuid == uuid:
                delIdx = i
                break

        if delIdx != None:
            self._primitives.pop(i).remove_from_cache()
            self.updated_attribute('primitives','delete')

    def get_primitive(self, uuid):
        for p in self.primitives:
            if p.uuid == uuid:
                return p
        return None

    def set(self, dct):

        if 'primitives' in dct.keys():
            self.primitives = [NodeParser(p) for p in dct['primitives']]

        super(Hierarchical,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):
        for p in self._primitives:
            p.remove_from_cache()

        super(Hierarchical,self).remove_from_cache()

    def add_to_cache(self):
        for p in self._primitives:
            p.add_to_cache()

        super(Hierarchical,self).add_to_cache()

    '''
    Children methods
    '''

    def delete_child(self, uuid):
        success = True

        if uuid in [p.uuid for p in self.primitives]:
            self.delete_primitive(uuid)
        else:
            success = False

        return success

    '''
    Update Methods
    '''

    def late_construct_update(self):

        for p in self.primitives:
            p.late_construct_update()

        super(Hierarchical,self).late_construct_update()

    def deep_update(self):

        for p in self.primitives:
            p.deep_update()

        super(Hierarchical,self).deep_update()

        self.updated_attribute('primitives','update')

    def shallow_update(self):
        super(Hierarchical,self).shallow_update()

        self.updated_attribute('primitives','update')

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        hooks.active_primitive = self

        if not self.uuid in hooks.state.keys():
            hooks.state[self.uuid] = { 'index': 0 }

        next = None
        index = hooks.state[self.uuid]['index']
        if index < len(self.primitives):
            next = self.primitives[index]
            hooks.state[self.uuid]['index'] = index + 1
        else:
            next = self.parent
            del hooks.state[self.uuid]
        return next

    def realtime_execution(self, hooks):
        return self.symbolic_execution(hooks)
