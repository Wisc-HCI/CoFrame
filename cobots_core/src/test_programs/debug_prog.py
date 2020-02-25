
from cobots_datatypes.node import *
from cobots_datatypes.task import *
from cobots_datatypes.cache import *
from cobots_datatypes.trace import *
from cobots_datatypes.context import *
from cobots_datatypes.program import *
from cobots_datatypes.geometry import *
from cobots_datatypes.location import *
from cobots_datatypes.waypoint import *
from cobots_datatypes.primitive import *
from cobots_datatypes.trajectory import *

def generate():
    # Define Program
    prog = Program()

    # Define and add location
    home_loc = Location()
    pick_stock_loc = Location()
    place_stock_loc = Location()
    retract_loc = Location()
    pick_final_loc = Location()
    place_final_loc = Location()

    prog.context.add_location(home_loc)
    prog.context.add_location(pick_stock_loc)
    prog.context.add_location(place_stock_loc)
    prog.context.add_location(retract_loc)
    prog.context.add_location(pick_final_loc)
    prog.context.add_location(place_final_loc)

    # Define and add machines
    cnc = Machine()

    prog.context.add_machine(cnc)

    # Define and add primitives
    prog.add_primitive(Initialize(
        homeLocUuid=home_loc.uuid,
        machineUuid=cnc.uuid))

    prog.add_primitive(Loop(
        primitives=[
            PickAndPlace(
                startLocUuid=home_loc.uuid,
                pickLocUuid=pick_stock_loc.uuid,
                placeLocUuid=place_stock_loc.uuid),
            MoveTrajectory(
                startLocUuid=place_stock_loc.uuid,
                endLocUuid=retract_loc.uuid),
            MachineBlockingProcess(
                machineUuid=cnc.uuid),
            PickAndPlace(
                startLocUuid=retract_loc.uuid,
                pickLocUuid=pick_final_loc.uuid,
                placeLocUuid=place_final_loc.uuid)
        ]
    ))

    return prog
