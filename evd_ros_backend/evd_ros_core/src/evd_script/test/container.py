
from ..node import Node


class Container(Node):

    '''
    Data structure methods
    '''

    def __init__(self, item_type, values=[], type='', name='', uuid=None, parent=None, append_type=True):

        self._values = None
        self._item_type = None

        super(Container,self).__init__(
            type='container<{}>.'.format(itemType) + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.values = values
        self.item_type = item_type

    def to_dct(self):
        msg = super(Container,self).to_dct()
        msg.update({
            'values': [x.to_dct() for x in self.values],
            'item_type': self.item_type
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        from ..utility_functions import NodeParser

        return cls(
            item_type=dct['item_type'],
            values=[NodeParser(x) for x in dct['values']],
            type=dct['type'],
            append_type=False,
            name=dct['name'],
            uuid=dct['uuid']
        )

    '''
    Data accessor/modifier methods
    '''

    @property
    def values(self):
        return self._values

    @values.setter
    def values(self, value):
        if self._values != value:

            for v in self._values:
                v.remove_from_cache()

            self._values = values
            for v in self._values:
                v.parent = self

            self.updated_attribute('values','set')

    @property
    def item_type(self):
        return _item_type

    @item_type.setter
    def item_type(self, value):
        if self._item_type != value:
            self._item_type = value
            self.updated_attribute('item_type','set')

    def add(self, value):
        value.parent = self
        self._values.append(value)
        self.updated_attribute('values','add',value.uuid)

    def get(self, uuid):
        return self._values[self.search_for_index(uuid)]

    def delete(self, uuid):
        idx = self.search_for_index(uuid)
        if idx == -1:
            raise Exception('Value not in container')

        self._values[idx].remove_from_cache()
        self._values.pop(idx)

        self.updated_attribute('values','delete',uuid)

    def search_for_index(self, uuid):
        idx = -1
        for i in range(0,len(self.values)):
            if self.values[i].uuid == uuid:
                idx = i
                break
        return idx

    def set(self, dct):
        from ..utility_functions import NodeParser

        if 'item_type' in dct.keys():
            self.item_type = dct.get('item_type')

        if 'values' in dct.keys():
            self.values = [NodeParser(x) for x in dct['values']]

        super(Container,self).set(dct)

    '''
    Cache Methods
    '''

    def remove_from_cache(self):
        for v in self._values:
            v.remove_from_cache()

        super(Container,self).remove_from_cache()

    def add_to_cache(self):
        for v in self._values:
            v.add_to_cache()

        super(Container,self).add_to_cache()

    '''
    Children Methods
    '''

    def delete_child(self, uuid):
        success  = False

        if uuid in [v.uuid for v in self._values]:
            self.delete(uuid)
            success = True

        return success

    '''
    Update Methods
    '''

    def late_construct_update(self):

        for v in self.values:
            v.late_construct_update()

        super(Container,self).late_construct_update()

    def deep_update(self):

        for v in self.values:
            v.deep_update()

        super(Container,self).deep_update()

        self.updated_attribute('values', 'update')
        self.updated_attribute('item_type', 'update')

    def shallow_update(self):
        super(Container,self).shallow_update()

        self.updated_attribute('values', 'update')
        self.updated_attribute('item_type', 'update')
