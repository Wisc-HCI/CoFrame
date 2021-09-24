#!/usr/bin/env python3

'''
This application is used to stress the educational dimension of EvD
'''

from scipy import interpolate
from evd_script import *

# NOTE: for future, we should probably think about hierarchical UUID structures.
MACHINE_UUID_3D_PRINTER = '3d-printer-machine-uuid'
MACHINE_UUID_ASSEMBLY_JIG = 'assembly-jig-machine-uuid'
MACHINE_UUID_BLADE_CONVEYOR = 'blade-conveyor-machine-uuid'
MACHINE_UUID_BLADE_FEEDER = 'blade-feeder-machine-uuid'
MACHINE_UUID_KNIFE_CONVEYOR = 'knife-conveyor-machine-uuid'
MACHINE_UUID_KNIFE_FEEDER = 'knife-feeder-machine-uuid'
ROBOT_UUID_UR3E = 'ur3e-robot-uuid'

#seconds (not realistic but ¯\_(ツ)_/¯)
PROCESS_TIME_3D_PRINTER = 5
PROCESS_TIME_KNIFE_FEEDER = 0
PROCESS_TIME_KNIFE_CONVEYOR = 5
PROCESS_TIME_BLADE_FEEDER = 0
PROCESS_TIME_BLADE_CONVEYOR = 5
PROCESS_TIME_ASSEMBLY_JIG = 0


def generate():
    
    cache = get_evd_cache_obj()
    prog = Program()

    #===========================================================================
    # Define Safety Environment
    #===========================================================================

    # Define Reach Sphere
    prog.environment.reach_sphere =  ReachSphere(
        radius=0.8, 
        link="simulated_base_link", 
        offset=Position(0,0,0.15))

    # Define Pinch Points
    prog.environment.pinch_points = [
        PinchPoint(
            link='simulated_shoulder_link', 
            radius=0.075, 
            length=0.2, 
            offset=Position.from_axis('z',-0.05)
        ),
        PinchPoint(
            link='simulated_upper_arm_link', 
            radius=0.075, 
            length=0.2, 
            offset=Position.from_axis('z',0.075)
        ),
        PinchPoint(
            link='simulated_forearm_link', 
            radius=0.075, 
            length=0.2, 
            offset=Position.from_axis('z',0.075)
        ),
        PinchPoint(
            link='simulated_wrist_1_link', 
            radius=0.06, 
            length=0.17, 
            offset=Position.from_axis('z',-0.05)
        ),
        PinchPoint(
            link='simulated_wrist_3_link', 
            radius=0.1, 
            length=0.16, 
            offset=Position.from_axis('z',0.1)
        )
    ]

    # Define Collison
    printerCollisionMesh = CollisionMesh(
        name='3D Printer Collision Mesh',
        link='3d_printer_link', 
        mesh_id='package://evd_ros_tasks/description/meshes/collision/3d_printer.stl')
    pedestalCollisionMesh = CollisionMesh(
        name="Pedestal Collision Mesh", 
        link='ur3e_pedestal_link', 
        mesh_id='package://evd_ros_tasks/description/meshes/collision/pedestal.stl')
    tableCollisionMesh = CollisionMesh(
        name="Table Collision Mesh", 
        link='table_link', 
        mesh_id='package://evd_ros_tasks/description/meshes/collision/table.stl')
    assemblyJigCollisionMesh = CollisionMesh(
        name="Assembly Jig Collision Mesh",
        link="assembly_jig_link",
        mesh_id="package://evd_ros_tasks/description/meshes/collision/assembly_jig.stl")
    bladeConveyorCollisionMesh = CollisionMesh(
        name="Blade Conveyor Collision Mesh",
        link="blade_conveyor_link",
        mesh_id="package://evd_ros_tasks/description/meshes/collision/conveyor.stl")
    bladeFeederCollisionMesh = CollisionMesh(
        name="Blade Feeder Collision Mesh",
        link="blade_feeder_link",
        mesh_id="package://evd_ros_tasks/description/meshes/collision/blade_feeder.stl")
    knifeConveyorCollisionMesh = CollisionMesh(
        name="Knife Conveyor Collision Mesh",
        link="knife_conveyor_link",
        mesh_id="package://evd_ros_tasks/description/meshes/collision/conveyor.stl")
    knifeFeederCollisionMesh = CollisionMesh(
        name="Knife Feeder Collision Mesh",
        link="knife_feeder_link",
        mesh_id="package://evd_ros_tasks/description/meshes/collision/knife_feeder.stl")
    prog.environment.collision_meshes = [
        printerCollisionMesh,
        pedestalCollisionMesh,
        tableCollisionMesh,
        assemblyJigCollisionMesh,
        bladeConveyorCollisionMesh,
        bladeFeederCollisionMesh,
        knifeConveyorCollisionMesh,
        knifeFeederCollisionMesh
    ]
    
    # Define Occupancy
    robotOccupancyZone = OccupancyZone(
        name="Robot Occupancy Zone", 
        occupancyType=OccupancyZone.ROBOT_TYPE, 
        sclX=1.6, sclZ=1.2, height=-0.8)
    humanWorkspaceOccupancyZone = OccupancyZone(
        name="Human Workspace Occupancy Zone", 
        occupancyType=OccupancyZone.HUMAN_TYPE, 
        posZ=1, sclX=2, height=-0.8)
    humanCorridorOccupancyZone = OccupancyZone(
        name="Human Corridor Occupancy Zone", 
        occupancyType=OccupancyZone.HUMAN_TYPE, 
        posZ=-1, sclX=2, height=-0.8)
    prog.environment.occupancy_zones = [
        robotOccupancyZone,
        humanWorkspaceOccupancyZone,
        humanCorridorOccupancyZone
    ]

    #===========================================================================
    # Define Machine Relevant Data
    #===========================================================================

    # Defining thing types needed for machine
    leftHandleThingType = ThingType(
        name='Left Handle',
        is_safe=True,
        weight=0,
        mesh_id='package://evd_ros_tasks/description/markers/left_handle.stl')
    rightHandleThingType = ThingType(
        name='Right Handle',
        is_safe=True,
        weight=0,
        mesh_id='package://evd_ros_tasks/description/markers/right_handle.stl')
    bladeThingType = ThingType(
        name='Blade',
        is_safe=False,
        weight=0,
        mesh_id='package://evd_ros_tasks/description/markers/blade.stl')
    transportJigThingType = ThingType(
        name='Transport Jig',
        is_safe=True,
        weight=0,
        mesh_id='package://evd_ros_tasks/description/markers/tranport_jig.stl')
    bladeWithTransportJigThingType = ThingType(
        name='Blade With Transport Jig',
        is_safe=True,
        weight=0,
        mesh_id='package://evd_ros_tasks/description/markers/blade_with_transport_jig.stl')
    knifeThingType= ThingType(
        name='Knife',
        is_safe=False,
        weight=0,
        mesh_id='package://evd_ros_tasks/description/markers/knife.stl')
    knifeWithTransportJigThingType = ThingType(
        name='Knife With Transport Jig',
        is_safe=False,
        weight=0,
        mesh_id='package://evd_ros_tasks/description/markers/knife_with_transport_jig.stl')
    prog.environment.thing_types = [
        leftHandleThingType,
        rightHandleThingType,
        bladeThingType,
        transportJigThingType,
        bladeWithTransportJigThingType,
        knifeThingType,
        knifeWithTransportJigThingType
    ]

    # Create placeholder things generated by machines
    leftHandlePlaceholder = Placeholder(
        pending_node_dct=Thing(
            thing_type_uuid=leftHandleThingType.uuid, 
            name='Left Handle Thing'
        ).to_dct(),
        pending_fields=[
            'position',
            'orientation'
        ])
    rightHandlePlaceholder = Placeholder(
        pending_node_dct=Thing(
            thing_type_uuid=rightHandleThingType.uuid, 
            name='Right Handle Thing'
        ).to_dct(),
        pending_fields=[
            'position',
            'orientation'
        ])
    transportJigPlaceholder = Placeholder(
        pending_node_dct=Thing(
            thing_type_uuid=transportJigThingType.uuid, 
            name='Transport Jig Placeholder',
            position=Position(0,0,0),
            orientation=Orientation.Identity()
        ).to_dct(),
        pending_fields=[])
    bladePlaceholder = Placeholder(
        pending_node_dct=Thing(
            thing_type_uuid=bladeThingType.uuid,
            name='Blade Thing'
        ).to_dct(),
        pending_fields=[
            'position',
            'orientation'
        ])
    bladeWithTransportJigPlaceholder = Placeholder(
        pending_node_dct=Thing(
            thing_type_uuid=bladeWithTransportJigThingType.uuid,
            name='Blade with Transport Jig Thing'
        ).to_dct(),
        pending_fields=[
            'position',
            'orientation'
        ])
    knifePlaceholder = Placeholder(
        pending_node_dct=Thing(
            thing_type_uuid=knifeThingType.uuid,
            name='Knife Thing'
        ).to_dct(),
        pending_fields=[
            'position',
            'orientation'
        ])
    knifeWithTransportJigPlaceholder = Placeholder(
        pending_node_dct=Thing(
            thing_type_uuid=knifeWithTransportJigThingType.uuid,
            name='Knife with Transport Jig Thing'
        ).to_dct(),
        pending_fields=[
            'position',
            'orientation'
        ])
    prog.environment.placeholders = [
        leftHandlePlaceholder,
        rightHandlePlaceholder,
        transportJigPlaceholder,
        bladePlaceholder,
        bladeWithTransportJigPlaceholder,
        knifePlaceholder,
        knifeWithTransportJigPlaceholder
    ]

    # Define Regions
    leftHandleRegion = CubeRegion(
        link='3d_printer_link', 
        center_position=Position(0,0,0),
        center_orientation=Orientation.Identity(),
        uncertainty_x=0.01,
        uncertainty_y=0.01,
        uncertainty_z=0.01)
    rightHandleRegion = CubeRegion(
        link='3d_printer_link', 
        center_position=Position(0,0,0),
        center_orientation=Orientation.Identity(),
        uncertainty_x=0.01,
        uncertainty_y=0.01,
        uncertainty_z=0.01)
    bladeFeederRegion = CubeRegion(
        link='blade_feeder_link',
        center_position=Position(0,0,0),
        center_orientation=Orientation.Identity(),
        uncertainty_x=0.01,
        uncertainty_y=0.01,
        uncertainty_z=0.01)
    knifeFeederRegion = CubeRegion(
        link='knife_feeder_link',
        center_position=Position(0,0,0),
        center_orientation=Orientation.Identity(),
        uncertainty_x=0.01,
        uncertainty_y=0.01,
        uncertainty_z=0.01)
    bladeAssemblyRegion = CubeRegion(
        link='assembly_jig_link',
        center_position=Position(0,0,0),
        center_orientation=Orientation.Identity(),
        uncertainty_x=0.01,
        uncertainty_y=0.01,
        uncertainty_z=0.01)
    prog.environment.regions = [
        leftHandleRegion,
        rightHandleRegion,
        bladeFeederRegion,
        knifeFeederRegion,
        bladeAssemblyRegion
    ]

    # Define Machines
    printerMachine = Machine(
        name='3D Printer Machine',
        uuid=MACHINE_UUID_3D_PRINTER,
        inputs={},
        outputs={
            leftHandleThingType.uuid: [{
                    'region_uuid': leftHandleRegion.uuid,
                    'quantity': 1,
                    'placeholder_uuids': [
                        leftHandlePlaceholder.uuid
                    ]
            }],
            rightHandleThingType.uuid: [{
                    'region_uuid': rightHandleRegion.uuid,
                    'quantity': 1,
                    'placeholder_uuids':[
                        rightHandlePlaceholder.uuid
                    ]
            }]
        },   
        passive=False,
        process_time=PROCESS_TIME_3D_PRINTER,
        link='3d_printer_link', 
        mesh_id='package://evd_ros_tasks/description/meshes/visual/3d_printer.stl',
        collision_mesh_uuid=printerCollisionMesh.uuid)
    assemblyJigUnsafeMachine = Machine(
        name="Assembly Jig Machine",
        uuid=MACHINE_UUID_ASSEMBLY_JIG+'_unsafe',
        inputs={
            bladeThingType.uuid: [{
                'region_uuid': bladeAssemblyRegion.uuid,
                'quantity': 1
            }],
            leftHandleThingType.uuid: [{
                'region_uuid': bladeAssemblyRegion.uuid,
                'quantity': 1
            }],
            rightHandleThingType.uuid: [{
                'region_uuid': bladeAssemblyRegion.uuid,
                'quantity': 1
            }]
        },
        outputs={
            knifeThingType.uuid: [{
                'region_uuid': bladeAssemblyRegion.uuid,
                'quantity': 1,
                'placeholder_uuids': [
                    knifePlaceholder.uuid
                ]
            }]
        },
        passive=True,
        process_time=PROCESS_TIME_ASSEMBLY_JIG,
        link='assembly_jig_link',
        mesh_id='package://evd_ros_tasks/description/meshes/visual/assembly_jig.stl',
        collision_mesh_uuid=assemblyJigCollisionMesh.uuid)
    assemblyJigSafeMachine = Machine(
        name="Assembly Jig Machine",
        uuid=MACHINE_UUID_ASSEMBLY_JIG+'_safe',
        inputs={
            bladeWithTransportJigThingType.uuid: [{
                'region_uuid': bladeAssemblyRegion.uuid,
                'quantity': 1
            }],
            leftHandleThingType.uuid: [{
                'region_uuid': bladeAssemblyRegion.uuid,
                'quantity': 1
            }],
            rightHandleThingType.uuid: [{
                'region_uuid': bladeAssemblyRegion.uuid,
                'quantity': 1
            }]
        },
        outputs={
            knifeWithTransportJigThingType.uuid: [{
                'region_uuid': bladeAssemblyRegion.uuid,
                'quantity': 1,
                'placeholder_uuids': [
                    knifeWithTransportJigPlaceholder.uuid
                ]
            }]
        },
        passive=True,
        process_time=PROCESS_TIME_ASSEMBLY_JIG,
        link='assembly_jig_link',
        mesh_id='package://evd_ros_tasks/description/meshes/visual/assembly_jig.stl',
        collision_mesh_uuid=assemblyJigCollisionMesh.uuid)
    bladeConveyorMachine = Machine(
        name="Blade Conveyor Machine",
        uuid=MACHINE_UUID_BLADE_CONVEYOR,
        inputs={},
        outputs={
            bladeThingType.uuid: [{
                'region_uuid': bladeFeederRegion.uuid,
                'quantity': 1,
                'placeholder_uuids': [
                    bladePlaceholder.uuid
                ]
            }]
        },
        passive=False,
        process_time=PROCESS_TIME_BLADE_CONVEYOR,
        link='blade_conveyor_link',
        mesh_id='package://evd_ros_tasks/description/meshes/visual/conveyor.stl',
        collision_mesh_uuid=bladeConveyorCollisionMesh.uuid)
    bladeFeederMachine = Machine(
        name="Blade Feeder Machine",
        uuid=MACHINE_UUID_BLADE_FEEDER,
        inputs={
            bladeThingType.uuid: [{
                'region_uuid': bladeFeederRegion.uuid,
                'quantity': 1
            }]
        },
        outputs={
            bladeWithTransportJigThingType.uuid: [{
                'region_uuid': bladeFeederRegion.uuid,
                'quantity': 1,
                'placeholder_uuids': [
                    knifeWithTransportJigPlaceholder.uuid
                ]
            }]
        },
        passive=False,
        process_time=PROCESS_TIME_BLADE_FEEDER,
        link='blade_feeder_link',
        mesh_id='package://evd_ros_tasks/description/meshes/visual/blade_feeder.stl',
        collision_mesh_uuid=bladeFeederCollisionMesh.uuid)
    knifeFeederMachine = Machine(
        name="Knife Feeder Machine",
        uuid=MACHINE_UUID_KNIFE_FEEDER,
        inputs={
            knifeWithTransportJigThingType.uuid: [{
                'region_uuid': knifeFeederRegion.uuid,
                'quality': 1
            }]
        },
        outputs={
            knifeThingType.uuid: [{
                'region_uuid': knifeFeederRegion.uuid,
                'quality': 1,
                'placeholder_uuids': [
                    knifePlaceholder.uuid
                ]
            }]
        },
        passive=True,
        process_time=PROCESS_TIME_KNIFE_FEEDER,
        link='knife_feeder_link',
        mesh_id='package://evd_ros_tasks/description/meshes/visual/knife_feeder.stl',
        collision_mesh_uuid=knifeFeederCollisionMesh.uuid)
    knifeConveyorMachine = Machine(
        name="Blade Conveyor Machine",
        uuid=MACHINE_UUID_KNIFE_CONVEYOR,
        inputs={
            knifeThingType.uuid: [{
                'region_uuid': knifeFeederRegion.uuid,
                'quantity': 1
            }]
        },
        passive=False,
        outputs={},
        process_time=PROCESS_TIME_KNIFE_CONVEYOR,
        link='knife_conveyor_link',
        mesh_id='package://evd_ros_tasks/description/meshes/visual/conveyor.stl',
        collision_mesh_uuid=knifeConveyorCollisionMesh.uuid)
    prog.environment.machines = [
        printerMachine,
        assemblyJigUnsafeMachine,
        assemblyJigSafeMachine,
        bladeConveyorMachine,
        bladeFeederMachine,
        knifeConveyorMachine,
        knifeFeederMachine
    ]

    #===========================================================================
    # Define Program Data (Waypoints, Locations, Trajectories)
    #===========================================================================

    #TODO

    #===========================================================================
    # Define Program Operation (Primitives, Skills, Etc.)
    #===========================================================================

    #TODO

    #===========================================================================
    # Done :)
    #===========================================================================

    prog.late_construct_update() # This will force the entire program to undergo patch/repair
    return prog


if __name__ == "__main__":
    import rospy
    rospy.init_node('test_node_application')

    program = generate()

    import json
    with open('knife_task_exported.json','w+') as f:
        json.dump(program.to_dct(), f, indent=4)