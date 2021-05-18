'''
Acts as a lazy import for the real NodeParser

This is necessary importing it at the start creates a set of cycles in
the dependency graph. Python chokes when trying to solve it.
'''

def NodeParser(dct, no_cache=False, enforce_type=None):
    from .utility_functions import NodeParser as RealNP

    return RealNP(dct,no_cache,enforce_type)