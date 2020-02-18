from primitive import Primitive


class Task(Primitive):

    def __init__(self, type='generic' name='', uuid=None):
        Primitive.__init__(self,'graph.'+type,name,uuid)

        #TODO generic graph structure

    def to_dct(self):
        return {
            'type': self._type,
            'uuid': self._uuid,
            'name': self.name
        }

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            uuid=dct['uuid']
        )


class CloseGripper(Task):
    pass #TODO


class OpenGripper(Task):
    pass #TODO


class PickAndPlaceTask(Task):
    pass #TODO


class RetractTask(Task):
    pass #TODO


class InitializeTask(Task):
    pass #TODO


class CNCProcessTask(Task):
    pass #TODO


class Loop(Task):
    pass #TODO
