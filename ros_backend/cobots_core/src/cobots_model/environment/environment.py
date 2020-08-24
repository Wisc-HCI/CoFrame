from ..node import Node

class Environment(Node):

    def __init__(self):
        pass

    def to_dct(self):
        return {}

    def set(self, dct):
        pass #TODO write this

    @classmethod
    def from_dct(self, dct):
        return cls()
