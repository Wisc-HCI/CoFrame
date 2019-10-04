from abstract import Abstract

class Waypoint(Abstract):

    def __init__(self):
        pass

    def to_dct(self):
        return {}

    @classmethod
    def from_dct(cls):
        return cls()
