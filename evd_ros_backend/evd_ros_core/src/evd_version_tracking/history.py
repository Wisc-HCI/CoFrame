'''
Collection of HistoryEntry objects.
'''


class History(object):

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

    @property
    def entries(self):
        return self._history

    def get_entry(self,version):
        for entry in self._history:
            if entry.version == version:
                return entry
        raise Exception('No entry with that version tag found')

    def to_dct(self):
        return {
            'depth': self._history_depth,
            'entries': [e.to_dct() for e in self.entries]
        }

    @classmethod
    def from_dct(cls, dct):
        obj = cls(dct['depth'])
        obj._history = [HistoryEntry.from_dct(e) for e in dct['entries']]
        return obj
