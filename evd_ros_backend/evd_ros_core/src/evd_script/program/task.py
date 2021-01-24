from .primitive import Primitive


class Task(Primitive):

    '''
    Data structure methods
    '''

    def __init__(self, primitives=[], type='', name='', uuid=None, parent=None, append_type=True):

        self._primitives = []

        super(Task,self).__init__(
            type='task.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.primitives = primitives

    def to_dct(self):
        msg = super(Task,self).to_dct()
        msg.update({
            'primitives': [p.to_dct() for p in self.primitives]
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        from ..utility_functions import NodeParser

        return cls(
            name=dct['name'],
            type=dct['type'],
            append_type=False,
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
        self.updated_attribute('primitives','set')

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
        from ..utility_functions import NodeParser

        if 'primitives' in dct.keys():
            self.primitives = [NodeParser(p) for p in dct['primitives']]

        super(Task,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):
        for p in self._primitives:
            p.remove_from_cache()

        super(Task,self).remove_from_cache()

    def add_to_cache(self):
        for p in self._primitives:
            p.add_to_cache()

        super(Task,self).add_to_cache()

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

        super(Task,self).late_construct_update()

    def deep_update(self):

        for p in self.primitives:
            p.deep_update()

        super(Task,self).deep_update()

        self.updated_attribute('primitives','update')

    def shallow_update(self):
        super(Task,self).shallow_update()

        self.updated_attribute('primitives','update')
