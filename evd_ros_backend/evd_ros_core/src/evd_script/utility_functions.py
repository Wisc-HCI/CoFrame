from .cache import *

from .data import DataNodeParser
from .environment import EnvironmentNodeParser
from .program import ProgramNodeParser
from .test import TestNodeParser
from .node import Node


def NodeParser(dct):

    ## Check if object already in the cache
    try:
        node = get_evd_cache_obj().get(dct['uuid'])
    except:
        node = None

    if node != None:
        return node

    ## Must create a new object
    type = dct["type"].split('.')
    exactType = type[len(type) - 2]

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

    if exactType == "node":
        node = Node.from_dct(dct)
    else:
        raise Exception('Could not parse object supplied with type: {}'.format(exactType))

    return node


def get_exact_type(node):
    type = node.type.split('.')
    exactType = type[len(type) - 2]
    return exactType
