from .cache import *
from .orphans import *

from .data_nodes import *
from .program_nodes import *
from .environment_nodes import *
from .test_nodes import *

from .node import Node
from .context import Context
from .program import Program
from .node_parser import NodeParser
from .environment import Environment
from .attribute_trace_processor import AttributeTraceProcessor
from .visualizable import VisualizeMarker, VisualizeMarkers, ColorTable

ALL_NODES_TYPE = '<all>'
ALL_PRIMITIVES_TYPES = '<all-primitives>'
ALL_REGION_TYPES = '<all-regions>'
ALL_CONDITIONS_TYPES = '<all-conditions>'
ALL_SKILLS_TYPES = '<all-skills>'

LOCATION_OR_WAYPOINT = '<location-or-waypoint>'

STRING_TYPE = '<string>'
NUMBER_TYPE = '<number>'
BOOLEAN_TYPE = '<boolean>'
ENUM_TYPE = '<enum>'
ARBITRARY_OBJ_TYPE = '<arbitrary-obj>'