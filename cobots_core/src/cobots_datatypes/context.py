
class Context(object):

    def __init__(self):
        self.locations = {}

    def getLocation(self, uuid):
        return self.locations[uuid]
        #TODO need to update rest of model

    def addLocation(self, loc):
        self.locations[loc.uuid] = loc

    def deleteLocation(self, uuid):
        self.locations.pop(uuid)
        # TODO need to update rest of model
