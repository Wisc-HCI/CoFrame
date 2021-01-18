from .container import Container


def TestNodeParser(exactType, dct):

    node = None

    if 'container' in exactType and 'item_type' in dct.keys():
        if exactType == 'container<{0}>'.format(dct['item_type']):
            node = Container.from_dct(dct)
        else:
            raise Exception('Illegal container format when attempting to run node parser')
            
    return node
