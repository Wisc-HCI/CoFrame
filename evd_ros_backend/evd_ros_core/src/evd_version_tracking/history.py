

class History(object):
    #TODO do history lookup
    #TODO store buffer for future with undo

    def __init__(self, history_depth=10):
        self._history = []
        self._history_depth = history_depth

    def append(self, entry):
        self._history.append(entry)
        if len(self._history) > self._history_depth:
            self._history.pop(0)

    def get_current_entry(self):
        if len(self._history) > 0:
            return self._history[-1]
        else:
            return None

    def get_current_version(self):
        if len(self._history) > 0:
            return self._history[-1].version
        else:
            return None

    def get_previous_version(self):
        if len(self._history) > 1:
            return self._history[-2].version
        else:
            return None
