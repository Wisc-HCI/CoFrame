'''
Captures an instance of program state.

Used to revert changes in the data server.
'''


from .version_tag import VersionTag


class HistoryEntry(object):

    def __init__(self, action, change_dct, version_tag=None, snapshot_dct=None, source=None):
        self.action = action
        self.changes = change_dct
        self.snapshot = snapshot_dct

        if version_tag != None:
            self.version = version_tag
        else:
            if source != None:
                self.version = VersionTag(source)
            else:
                raise Exception("Must either provide version tag or source for new tag")

    def to_dct(self):
        return {
            'type': 'history-entry',
            'action': self.action,
            'change': self.changes,
            'version_tag': self.version.to_dct(),
            'snapshot': self.snapshot
        }

    @classmethod
    def from_dct(cls, dct):
        return cls(
            action=dct['action'],
            change_dct=dct['change'],
            version_tag=VersionTag.from_dct(dct['version_tag']),
            snapshot_dct=dct['snapshot']
        )
