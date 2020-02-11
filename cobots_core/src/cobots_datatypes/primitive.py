from abstract import Abstract


def PrimitiveFactory(dct):
    return None


class Primitive(Abstract):

    def __init__(self, type, name='', uuid=None):
        Abstract.__init__(self,'primitive.'+type,name,uuid)

    def to_dct(self):
        return {
            'type': self._type,
            'uuid': self._uuid,
            'name': self.name
        }

    @classmethod
    def from_dct(cls, dct):
        return cls(
            type=dct['type']
            name=dct['name'],
            uuid=dct['uuid']
        )

    def update(self, dct):
        pass #TODO


class Graph(Primitive):

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


class Break(Primitive):
    pass


class MovePrimitive(Primitive):

    def __init__(self, name='', uuid=None, startLoc_uuid=None, endLoc_uuid=None,
                 trajectory_uuids=[], runnableTraj_uuid=None):
        Primitive.__init__(self,'move',name,uuid)
        self.start_location_uuid = startLoc_uuid
        self.end_location_uuid = endLoc_uuid
        self.trajectory_uuids = trajectory_uuids
        self.runnable_trajectory_uuid = runnableTraj_uuid

    def to_dct(self):
        return  {
            'name': self.name,
            'type': self.type,
            'uuid': self.uuid,
            'start_location_uuid': self.start_location_uuid,
            'end_location_uuid': self.end_location_uuid,
            'trajectory_uuids': self.trajectory_uuids,
            'runnable_trajectory_uuid': self.runnable_trajectory_uuid
        }

    @classmethod
    def from_dct(cls, dct):
        if dct['type'] != 'move':
            raise Exception("Incorrect type provided {} is not move".format(dct['type']))
        return cls(
            name=dct['name'],
            uuid=dct['uuid'],
            startLoc_uuid=dct['start_location_uuid'],
            endLoc_uuid=dct['end_location_uuid'],
            trajectory_uuids=dct['trajectory_uuids'],
            runnableTraj_uuid=dct['runnable_trajectory_uuid']
        )


class DelayPrimitive(Primitive):

    def __init__(self, name='', uuid=None, duration=0):
        Primitive.__init__(self,'delay',name,uuid)
        self.duration = duration

    def to_dct(self):
        return {
            'name': self.name,
            'type': self.type,
            'uuid': self.uuid,
            'duration': self.duration
        }

    @classmethod
    def from_dct(cls, dct):
        if dct['type'] != 'delay':
            raise Exception("Incorrect type provided {} is not delay".format(dct['type']))
        return cls(
            name=dct['name'],
            uuid=dct['uuid'],
            duration = dct['duration']
        )


class GripPrimitive(Primitive):

    def __init__(self, name='', uuid=None, position=0, effort=0, speed=0):
        Primitive.__init__(self,'grip',name,uuid)
        self.position = position
        self.effort = effort
        self.speed = speed

    def to_dct(self):
        return {
            'name': self.name,
            'type': self.type,
            'uuid': self.uuid,
            'position': self.position,
            'effort': self.effort,
            'speed': self.speed
        }

    @classmethod
    def from_dct(cls, dct):
        if dct['type'] != 'grip':
            raise Exception("Incorrect type provided {} is not grip".format(dct['type']))
        return cls(
            name=dct['name'],
            uuid=dct['uuid'],
            position=dct['position'],
            effort=dct['effprt'],
            speed=dct['speed']
        )


class MachinePrimitive(Primitive):

    def __init__(self, action, name='', uuid=None):
        Primitive.__init__(self,'machine',name,uuid)

        self.action = action

    def to_dct(self):
        return {
            'name': self.name,
            'type': self.type,
            'uuid': self.uuid,
            'action': self.action
        }

    @classmethod
    def from_dct(cls, dct):
        if dct['type'] != 'machine':
            raise Exception("Incorrect type provided {} is not machine".format(dct['type']))
        return cls(
            name=dct['name'],
            uuid=dct['uuid'],
            action=dct['action']
        )
