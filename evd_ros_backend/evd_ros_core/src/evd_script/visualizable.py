from abc import ABCMeta, abstractmethod

from std_msgs.msg import ColorRGBA

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


class ColorTable:

    LOCATION_COLOR = ColorRGBA(173/255.0,216/255.0,230/255.0,0.5)
    WAYPOINT_COLOR = ColorRGBA(123/255.0,104/255.0,238/255.0,0.5)
    REGION_COLOR = ColorRGBA(0,0,1,0.5)
    THING_COLOR = ColorRGBA(0,0,1,1)
    TRAJECTORY_COLOR = ColorRGBA(1,1,1,0.5)

    OCCUPANCY_ZONE_ROBOT_COLOR = ColorRGBA(0,0.2,0,0.2)
    OCCUPANCY_ZONE_HUMAN_COLOR = ColorRGBA(0.2,0,0,0.2)

    TRACE_DATA_POINT_COLOR = ColorRGBA(255/255.0,255/255.0,255/255.0,0.5)
    TRACE_END_EFFECTOR_COLOR = ColorRGBA(0/255.0,255/255.0,0/255.0,0.5)
    TRACE_JOINT_COLOR = ColorRGBA(0/255.0,0/255.0,255/255.0,0.5)
    TRACE_TOOL_COLOR = ColorRGBA(255/255.0,0/255.0,0/255.0,0.5)
    TRACE_COMPONENT_COLOR = ColorRGBA(128/255.0,128/255.0,128/255.0,0.5)

    GOOD_COLOR = ColorRGBA(0,1,0,0.2)
    WARN_COLOR = ColorRGBA(0.5,0.5,0,0.2)
    ERROR_COLOR = ColorRGBA(1,0,0,0.2)
