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

    from .data import DataNodeParser
    node = DataNodeParser(exactType, dct)
    if node != None:
        return node

    from .environment import EnvironmentNodeParser
    node = EnvironmentNodeParser(exactType, dct)
    if node != None:
        return node

    from .program import ProgramNodeParser
    node = ProgramNodeParser(exactType, dct)
    if node != None:
        return node

    from .test import TestNodeParser
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
