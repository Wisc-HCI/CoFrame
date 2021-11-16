'''
Utility Functions defines the node parser behavior.

Node parser is the deserialization mechanism for EvD. It starts with
a cache search to idenitfy if a variant of the node already exists. If it
does then the state is updated with the provided serialized node. If no
node exists with that id and type in cache then a new node is constructed
using the serialized node data.

This new node creation is accomlished by walking the subparsers in each
namespace of EvDScript.

Warning!
Node parser works in a lazy import manner. This is done to prevent
python from choking on the dependency graph if imports are hoisted to
the start.
'''


def NodeParser(dct, no_cache=False, enforce_types=None):
    type = dct["type"].split('.')
    exactType = type[len(type) - 2]

    # Handle generalized types if a list is provided
    if enforce_types != None:
        # We don't support deserialization of basic types from here
        from .type_defs import STRING_TYPE, NUMBER_TYPE, BOOLEAN_TYPE, ENUM_TYPE, \
            ARBITRARY_OBJ_TYPE, PARAMETERS_FIELD_DCT
        if STRING_TYPE in enforce_types or NUMBER_TYPE in enforce_types \
                or BOOLEAN_TYPE in enforce_types or ENUM_TYPE in enforce_types \
                or ARBITRARY_OBJ_TYPE in enforce_types or PARAMETERS_FIELD_DCT in enforce_types:
            raise Exception('Basic types <string, numbers, booleans, enums, objs> cannot be parsed out!')

        # Expand out enforce list for generalized type descriptors
        from .type_defs import ALL_NODES_TYPE, ALL_PRIMITIVES_TYPES, ALL_REGION_TYPES, \
            ALL_CONDITIONS_TYPES, LOCATION_OR_WAYPOINT
        if ALL_NODES_TYPE in enforce_types:
            enforce_types = None # any node is valid so why enforce?
        
        elif ALL_PRIMITIVES_TYPES in enforce_types:
            from .program_nodes import primitive_library, Primitive, MachinePrimitive
            enforce_types.extend([x.type_string(trailing_delim=False) for x in primitive_library])
            enforce_types.extend([
                Primitive.type_string(trailing_delim=False),
                MachinePrimitive.type_string(trailing_delim=False)])

        elif ALL_REGION_TYPES in enforce_types:
            from .data_nodes.regions import Region, CubeRegion, SphereRegion
            enforce_types.extend([
                Region.type_string(trailing_delim=False),
                CubeRegion.type_string(trailing_delim=False),
                SphereRegion.type_string(trailing_delim=False)])
        
        elif ALL_CONDITIONS_TYPES in enforce_types:
            pass # Not supported right now
        
        elif LOCATION_OR_WAYPOINT in enforce_types:
            from .data_nodes import Location, Waypoint
            enforce_types.extend([
                Location.type_string(trailing_delim=False),
                Waypoint.type_string(trailing_delim=False)])

    # Enforce type of input matches expected type
    if enforce_types != None and exactType not in enforce_types:
        raise Exception('Type provided in dict does not match type enforced')

    #==========================================================================
    #   Search in Cache
    #==========================================================================

    ## Check if object already in the cache
    if not no_cache:
        from .cache import get_evd_cache_obj

        try:
            node = get_evd_cache_obj().get(dct['uuid'])
        except:
            node = None

        if node != None:

            # Enforce type of node matches the expected type
            type = node.type.split('.')
            exactType = type[len(type) - 2]
            if enforce_types != None and exactType not in enforce_types:
                raise Exception('Type of node found in cache does not match type enforced')

            # Repair node with input
            node.set(dct)
            return node

    #==========================================================================
    #   Create New Node
    #==========================================================================

    from .data_nodes import DataNodeParser
    node = DataNodeParser(exactType, dct)
    if node != None:
        return node

    from .environment_nodes import EnvironmentNodeParser
    node = EnvironmentNodeParser(exactType, dct)
    if node != None:
        return node

    from .program_nodes import ProgramNodeParser
    node = ProgramNodeParser(exactType, dct)
    if node != None:
        return node

    from .test_nodes import TestNodeParser
    node = TestNodeParser(exactType, dct)
    if node != None:
        return node

    # Top level nodes
    from .node import Node
    from .context import Context
    from .program import Program
    from .environment import Environment

    if exactType == Node.type_string(trailing_delim=False):
        node = Node.from_dct(dct)
    elif exactType == Context.type_string(trailing_delim=False):
        node = Context.from_dct(dct)
    elif exactType == Environment.type_string(trailing_delim=False):
        node = Environment.from_dct(dct)
    elif exactType == Program.type_string(trailing_delim=False):
        node = Program.from_dct(dct)
    else:
        raise Exception('Could not parse object supplied with type: {}'.format(exactType))

    return node
