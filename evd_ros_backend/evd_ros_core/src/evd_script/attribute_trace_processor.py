
class AttributeTraceProcessor:

    def __init__(self):
        self._subscribers = {}

    def subscribe_to_type(self, type, callback):
        if not type in self._subscribers.keys():
            self._subscribers[type] = []

        self._subscribers[type].append(callback)

    def unsubscribe_to_type(self, type, callback):
        if not type in self._subscribers.keys():
            return False

        if not callback in self._subscribers[type]:
            return False

        self._subscribers[type].remove(callback)
        return True

    def process_trace(attributeTrace):
        for entry in attributeTrace:
            if entry["type"] in self._subscribers.keys():
                for cb in self._subscribers[entry["type"]]:
                    cb(entry)
