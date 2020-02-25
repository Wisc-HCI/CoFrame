

class Cache(object):

    def __init__(self):
        self.data = {}

    def add(self, uuid, node):
        self.data[uuid] = node

    def remove(self, uuid):
        self.data.pop(uuid, None)

    def clear(self):
        self.data = {}

    def get(self, uuid):
        return self.get(uuid)
