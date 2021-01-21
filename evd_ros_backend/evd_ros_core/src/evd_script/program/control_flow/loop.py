from ..task import Task


class Loop(Task):

    '''
    Data structure methods
    '''

    def __init__(self, primitives=[], condition=None, type='', name='', uuid=None, parent=None,
                 append_type=True, context=None):
        self._condition = None

        super(Loop,self).__init__(
            type='loop.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            context=context,
            primitives=primitives)

        self.condition = condition

    def to_dct(self):
        msg = super(Loop,self).to_dct()
        msg.update({
            'condition': self.condition.to_dct() if self.condition != None else None
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        from ...utility_functions import NodeParser

        return cls(
            condition=NodeParser(dct['condition']) if dct['condition'] != None else None,
            name=dct['name'],
            uuid=dct['uuid'],
            type=dct['type'],
            append_type=False)

    '''
    Data accessor/modifier methods
    '''

    @property
    def condition(self):
        return self._condition

    @condition.setter
    def condition(self, value):
        if self._condition != value:

            if self._condition != None:
                self._condition.remove_from_cache()
            self._condition = value
            if self._condition != None:
                self._condition.parent = self

            self.updated_attribute('condition','set')

    def set(self, dct):

        if 'condition' in dct.keys():
            self.condition = NodeParser(dct['condition']) if dct['condition'] != None else None

        super(Loop,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):

        if self.condition != None:
            self.condition.remove_from_cache()

        super(Loop,self).remove_from_cache()

    def add_to_cache(self):

        if self.condition != None:
            self.condition.add_to_cache()

        super(Loop,self).add_to_cache()

    '''
    Update Methods
    '''

    def deep_update(self):

        self.condition.deep_update()

        super(Loop,self).deep_update()

        self.updated_attribute('condition','update')

    def shallow_update(self):

        super(Loop,self).shallow_update()

        self.updated_attribute('condition','update')
