from .region import Region
from .cube_region import CubeRegion
from .sphere_region import SphereRegion


def RegionsNodeParser(exactType, dct):
    node = None

    if exactType == Region.type_string(trailing_delim=False):
        node = Region.from_dct(dct)
    elif exactType == CubeRegion.type_string(trailing_delim=False):
        node = CubeRegion.from_dct(dct)
    elif exactType == SphereRegion.type_string(trailing_delim=False):
        node = SphereRegion.from_dct(dct)

    return node