from ..node import Node

class ReachSphere(Node):

    '''
    Constants
    '''

    GOOD_STATE = "good"
    WARN_STATE = "warn"
    ERROR_STATE = "error"

    '''
    Data structure methods
    '''

    def __init__(self, state = None, type='', name='', parent=None, uuid=None, append_type=True):
        self._state = None

        super(ReachSphere,self).__init__(
            type='reach-sphere.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.state = state if state != None else GOOD_STATE

    def to_dct(self):
        msg = super(ReachSphere,self).to_dct()
        msg.update({
            'state': self.state
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(state=dct['state'],
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    '''
    Data accessor/modifier methods
    '''

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        if self._state != value:
            if value != self.GOOD_STATE and value != self.WARN_STATE and value != self.ERROR_STATE:
                raise Exception('Invalid state provided')

            self._state = value
            self.updated_attribute('state','set')

    def set_state_good(self):
        self.state = self.GOOD_STATE

    def set_state_warn(self):
        self.state = self.WARN_STATE

    def set_state_error(self):
        self.state = self.ERROR_STATE

    def set(self, dct):
        if 'state' in dct.keys():
            self.state = dct['state']

        super(ReachSphere,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):
        super(ReachSphere,self).deep_update()

        self.updated_attribute('state','update')

    def shallow_update(self):
        super(ReachSphere,self).shallow_update()

        self.updated_attribute('state','update')
