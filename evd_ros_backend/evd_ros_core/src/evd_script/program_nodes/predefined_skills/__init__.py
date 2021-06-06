from .simple_pick_and_place import SimplePickAndPlace
from .machine_blocking_process import MachineBlockingProcess


predefined_skills_library = [
    MachineBlockingProcess,
    SimplePickAndPlace
]


def PredefinedSkillsNodeParser(exactType, dct):
    node = None

    if exactType == SimplePickAndPlace.type_string(trailing_delim=False):
        node = SimplePickAndPlace.from_dct(dct)
    elif exactType == MachineBlockingProcess.type_string(trailing_delim=False):
        node = MachineBlockingProcess.from_dct(dct)

    return node