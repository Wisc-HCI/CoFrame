export const COLLISION_MESHES = {
    'package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/Table/Table.stl':'package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/Table/Table.stl',
    'conveyor':'conveyor_collision',
    'conveyor_receiver':'conveyor_receiver_collision',
    'conveyor_dispatcher':'conveyor_dispatcher_collision',
    'package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/ur3e-Pedestal/Pedestal.stl':'package://evd_ros_tasks/tasks/3d_printer_machine_tending/collision_meshes/Pedestal.stl',
    'package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/MK2-Printer/MK2-Printer.stl':'package://evd_ros_tasks/tasks/3d_printer_machine_tending/collision_meshes/MK2-Printer.stl',
    'assembly_jig':'assembly_jig_collision',
    'package://ur_description/meshes/ur3/visual/base.dae':'package://ur_description/meshes/ur3/collision/base.stl',
    'package://ur_description/meshes/ur3/visual/shoulder.dae':'package://ur_description/meshes/ur3/collision/shoulder.stl',
    'package://ur_description/meshes/ur3/visual/upperarm.dae':'package://ur_description/meshes/ur3/collision/upperarm.stl',
    'package://ur_description/meshes/ur3/visual/forearm.dae':'package://ur_description/meshes/ur3/collision/forearm.stl',
    'package://ur_description/meshes/ur3/visual/wrist1.dae':'package://ur_description/meshes/ur3/collision/wrist1.stl',
    'package://ur_description/meshes/ur3/visual/wrist2.dae':'package://ur_description/meshes/ur3/collision/wrist2.stl',
    'package://ur_description/meshes/ur3/visual/wrist3.dae':'package://ur_description/meshes/ur3/collision/wrist3.stl',
    'package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae':'package://robotiq_85_description/meshes/collision/robotiq_85_base_link.stl',
    'package://robotiq_85_description/meshes/visual/robotiq_85_knuckle_link.dae':'package://robotiq_85_description/meshes/collision/robotiq_85_knuckle_link.stl',
    'package://robotiq_85_description/meshes/visual/robotiq_85_finger_link.dae':'package://robotiq_85_description/meshes/collision/robotiq_85_finger_link.stl',
    'package://robotiq_85_description/meshes/visual/robotiq_85_inner_knuckle_link.dae':'package://robotiq_85_description/meshes/collision/robotiq_85_inner_knuckle_link.stl',
    'package://robotiq_85_description/meshes/visual/robotiq_85_finger_tip_link.dae':'package://robotiq_85_description/meshes/collision/robotiq_85_finger_tip_link.stl'
}

export const EVD_MESH_LOOKUP = {
    "package://evd_ros_tasks/description/meshes/visual/3d_printer.stl":"package://evd_ros_tasks/tasks/3d_printer_machine_tending/models/MK2-Printer/MK2-Printer.stl",
    "package://evd_ros_tasks/description/meshes/visual/assembly_jig.stl":"assembly_jig",
    "package://evd_ros_tasks/description/meshes/visual/conveyor.stl":"conveyor",
    "package://evd_ros_tasks/description/meshes/visual/blade_feeder.stl":"conveyor_receiver",
    "package://evd_ros_tasks/description/meshes/visual/knife_feeder.stl":"conveyor_dispatcher",
    "package://evd_ros_tasks/description/markers/left_handle.stl":"handle_l",
    "package://evd_ros_tasks/description/markers/right_handle.stl":"handle_r",
    "package://evd_ros_tasks/description/markers/blade.stl":"blade",
    "package://evd_ros_tasks/description/markers/tranport_jig.stl":"transport_jig",
    "package://evd_ros_tasks/description/markers/blade_with_transport_jig.stl":"blade_with_transport_jig",
    "package://evd_ros_tasks/description/markers/knife_with_transport_jig.stl":"knife_with_transport_jig"
}