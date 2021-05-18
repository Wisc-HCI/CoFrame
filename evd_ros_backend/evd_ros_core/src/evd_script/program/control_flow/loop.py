from ..skill import Skill


class Loop(Skill):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'loop' + '.' if trailing_delim else ''

    @classmethod
    def full_type_string(cls):
        return Skill.full_type_string() + cls.type_string()

    def __init__(self, primitives=[], condition=None, type='', name='', uuid=None, parent=None,
                 append_type=True):
        self._condition = None

        super(Loop,self).__init__(
            type=Loop.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
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
            primitives=[NodeParser(p) for p in dct['primitives']],
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
            from ...utility_functions import NodeParser

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

    def late_construct_update(self):

        if self.condition != None:
            self.condition.late_construct_update()

        super(Loop,self).late_construct_update()

    def deep_update(self):

        if self.condition != None:
            self.condition.deep_update()

        super(Loop,self).deep_update()

        self.updated_attribute('condition','update')

    def shallow_update(self):

        super(Loop,self).shallow_update()

        self.updated_attribute('condition','update')

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        hooks.active_primitive = self

        run = True
        while run:
            for p in self.primitives:
                p.symbolic_execution(hooks)

            run = self.condition.symbolic_execution(hooks) if self.condition != None else True

    def realtime_execution(self, hooks):
        hooks.active_primitive = self

        run = True
        while run:
            for p in self.primitives:
                p.symbolic_execution(hooks)

            run = self.condition.symbolic_execution(hooks) if self.condition != None else True
