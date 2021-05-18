from .cache import *

from .data import DataNodeParser
from .environment import EnvironmentNodeParser
from .program import ProgramNodeParser
from .test import TestNodeParser
from .node import Node
from .context import Context


def NodeParser(dct, no_cache=False, enforce_type=None):

    type = dct["type"].split('.')
    exactType = type[len(type) - 2]

    if enforce_type != None and exactType != enforce_type:
        raise Exception('Type provided in dict does not match type enforced')

    ## Check if object already in the cache
    if not no_cache:
        try:
            node = get_evd_cache_obj().get(dct['uuid'])
        except:
            node = None

        if node != None:
            type = node.type.split('.')
            exactType = type[len(type) - 2]
            if enforce_type != None and exactType != enforce_type:
                raise Exception('Type of node found in cache does not match type enforced')

            return node

    ## Must create a new object
    node = DataNodeParser(exactType, dct)
    if node != None:
        return node

    node = EnvironmentNodeParser(exactType, dct)
    if node != None:
        return node

    node = ProgramNodeParser(exactType, dct)
    if node != None:
        return node

    node = TestNodeParser(exactType, dct)
    if node != None:
        return node

    if exactType == Node.type_string(trailing_delim=False):
        node = Node.from_dct(dct)
    elif exactType == Context.type_string(trailing_delim=False):
        node = Context.from_dct(dct)
    else:
        raise Exception('Could not parse object supplied with type: {}'.format(exactType))

    return node


def get_exact_type(node):
    type = node.type.split('.')
    exactType = type[len(type) - 2]
    return exactType
