'''
EvDScript's AST generates callback traces, which can be hooked into at the root program.

This is just a simple subscriber for the traces to route various in-app callbacks that 
you may want. For instance, your program may require tracking changes on traces. You can
then subscribe with a type=Trace.full_type_string() and your callback. When ever a change
happens that has the type field set as specified, your supplied callback will be invoked
and the entry in the trace will be passed.

This entry is defined as the node's child changed message.
'''

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

    def process_trace(self, attributeTrace):
        for entry in attributeTrace:
            if entry["type"] in self._subscribers.keys():
                for cb in self._subscribers[entry["type"]]:
                    cb(entry)
