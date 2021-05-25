'''
Loop is a sub-skill structure that allows for rerunning a list of nodes. 

At this point, loop only supports infinite looping but in the future conditional looping
is desired.

TODO support conditional looping
'''

from ..skill import Skill
from ...node_parser import NodeParser


class Loop(Skill):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'loop' + ('.' if trailing_delim else '')

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

        if not self.uuid in hooks.state.keys():
            hooks.state[self.uuid] = { 'index': 0, 'checked_cond': False }

        next = None
        if not hooks.state[self.uuid]['checked_cond']:
            # Check condition
            hooks.state[self.uuid]['checked_cond'] = True
            
            if self.condition != None:
                next = self.condition
            else:
                next = self # infinite loop

        elif hooks.state[self.uuid]['checked_cond'] and self.condition != None and not hooks.state[self.condition.uuid]['result']:
            # Exit based on condition (if no condition supplied then infinite loop)
            next = self.parent
            del hooks.state[self.uuid]
            del hooks.state[self.condition.uuid]

        else:
            # Run through inner contents of loop

            index = hooks.state[self.uuid]['index']
            if index < len(self.primitives):
                # Index through primitives in list
                next = self.primitives[index]
                hooks.state[self.uuid]['index'] = index + 1
            
            else:
                # Try next iteration of loop
                next = self
                hooks.state[self.uuid]['checked_cond'] = False

        return next

    def realtime_execution(self, hooks):
        return self.symbolic_execution(hooks)