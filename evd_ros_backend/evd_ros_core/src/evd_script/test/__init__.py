from .container import Container


def TestNodeParser(exactType, dct):

    node = None

    if 'container' in exactType and 'item_type' in dct.keys():
        if exactType == Container.type_string(dct['item_type'], trailing_delim=False):
            node = Container.from_dct(dct)
        else:
            raise Exception('Illegal container format when attempting to run node parser')

    return node
