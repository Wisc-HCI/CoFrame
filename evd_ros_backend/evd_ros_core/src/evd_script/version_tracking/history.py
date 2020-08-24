import uuid
import rospy

from evd_ros_core.msg import Version


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
            return self._history[-1].version_tag
        else:
            return None

    def get_previous_version(self):
        if len(self._history) > 1:
            return self._history[-2].version_tag
        else:
            return None


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


class VersionTag(object):

    def __init__(self, source, timestamp=None, uuid=None):
        if uuid != None:
            self.uuid = uuid
        else:
            self.uuid = self._generate_uuid('verison-tag')
        self.timestamp = timestamp if timestamp != None else rospy.Time.now().to_sec()
        self.source = source

    def to_dct(self):
        return {
            'type': 'version_tag',
            'uuid': self.uuid,
            'timestamp': self.timestamp,
            'source': self.source
        }

    def to_ros(self):
        msg = Version()
        msg.source = self.source
        msg.uuid = self.uuid
        msg.timestamp = rospy.Time.from_sec(self.timestamp)
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            source = dct['source'],
            timestamp = dct['timestamp'],
            uuid = dct['uuid']
        )

    @classmethod
    def from_ros(cls, msg):
        return cls(
            source = msg.source,
            timestamp = msg.timestamp.to_sec(),
            uuid = msg.uuid
        )

    def __eq__(self, other):
        try:
            return self.uuid == other.uuid
        except:
            return False

    @staticmethod
    def _generate_uuid(type):
        return '{}-py-{}'.format(type,uuid.uuid1().hex)
