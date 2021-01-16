from abc import ABCMeta, abstractmethod

# Please pick just one of these to implement per object as a general rule of thumb

class VisualizeMarker:
    __metaclass__ = ABCMeta

    @abstractmethod
    def to_ros_marker(self, frame_id, id=0):
        pass # Returns single marker


class VisualizeMarkers:
    __metaclass__ = ABCMeta

    @abstractmethod
    def to_ros_markers(self, frame_id, id_start=0):
        pass #Returns main_marker(s), secondary_markers, uuids_of_secondary_markers
