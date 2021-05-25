'''
The data server needs some way of determining truth. We can resolve change order
with uniquely identifiable timestamps. VersionTags perform this role by "tagging"
changes in the code with its origin, timestamp, and a uuid (not to be confused with
evdscript uuids).
'''


import uuid
import rospy

from evd_ros_core.msg import Version


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
