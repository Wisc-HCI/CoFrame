const KNIFE_TASK = {
    "type": "node.primitive.hierarchical.program.",
    "name": "Knife Assembly",
    "uuid": "primitive.hierarchical.program.-py-ec3c025a16a311ec82af00155d967c6c",
    "editable": true,
    "deleteable": true,
    "description": "",
    "parameters": {},
    "primitives": [],
    "environment": {
        "type": "node.context.environment.",
        "name": "",
        "uuid": "context.environment.-py-ec3c000216a311ec82af00155d967c6c",
        "editable": true,
        "deleteable": true,
        "description": "",
        "locations": [],
        "machines": [
            {
                "type": "node.machine.",
                "name": "3D Printer Machine",
                "uuid": "3d-printer-machine-uuid",
                "editable": true,
                "deleteable": true,
                "description": "",
                "inputs": {},
                "outputs": {
                    "thing-type.-py-ec3cb38016a311ec82af00155d967c6c": [
                        {
                            "region_uuid": "pose.region.cube-region.-py-ec3cdd2416a311ec82af00155d967c6c",
                            "quantity": 1,
                            "placeholder_uuids": [
                                "placeholder.-py-ec3cc0be16a311ec82af00155d967c6c"
                            ]
                        }
                    ],
                    "thing-type.-py-ec3cb43e16a311ec82af00155d967c6c": [
                        {
                            "region_uuid": "pose.region.cube-region.-py-ec3ce1de16a311ec82af00155d967c6c",
                            "quantity": 1,
                            "placeholder_uuids": [
                                "placeholder.-py-ec3cc4d816a311ec82af00155d967c6c"
                            ]
                        }
                    ]
                },
                "process_time": 5,
                "mesh_id": "package://evd_ros_tasks/description/meshes/visual/3d_printer.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-ec3cf79616a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "3d_printer_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-ec3cf86816a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0
                    },
                    "orientation": {
                        "type": "node.orientation.",
                        "name": "",
                        "uuid": "orientation.-py-ec3cf9c616a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "w": 1
                    }
                },
                "link": "3d_printer_link",
                "collision_mesh_uuid": "environment-node.collision-mesh.-py-ec3c6fc416a311ec82af00155d967c6c",
                "passive": false
            },
            {
                "type": "node.machine.",
                "name": "Assembly Jig Machine",
                "uuid": "assembly-jig-machine-uuid",
                "editable": true,
                "deleteable": true,
                "description": "",
                "inputs": {
                    "thing-type.-py-ec3cb60016a311ec82af00155d967c6c": {
                        "region_uuid": "pose.region.cube-region.-py-ec3ceb0c16a311ec82af00155d967c6c",
                        "quantity": 1
                    },
                    "thing-type.-py-ec3cb38016a311ec82af00155d967c6c": {
                        "region_uuid": "pose.region.cube-region.-py-ec3ceb0c16a311ec82af00155d967c6c",
                        "quantity": 1
                    },
                    "thing-type.-py-ec3cb43e16a311ec82af00155d967c6c": {
                        "region_uuid": "pose.region.cube-region.-py-ec3ceb0c16a311ec82af00155d967c6c",
                        "quantity": 1
                    }
                },
                "outputs": {
                    "thing-type.-py-ec3cb70416a311ec82af00155d967c6c": {
                        "region_uuid": "pose.region.cube-region.-py-ec3ceb0c16a311ec82af00155d967c6c",
                        "quantity": 1,
                        "placeholder_uuids": [
                            "placeholder.-py-ec3cdaa416a311ec82af00155d967c6c"
                        ]
                    }
                },
                "process_time": 0,
                "mesh_id": "package://evd_ros_tasks/description/meshes/visual/assembly_jig.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-ec3d019616a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "assembly_jig_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-ec3d022c16a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0
                    },
                    "orientation": {
                        "type": "node.orientation.",
                        "name": "",
                        "uuid": "orientation.-py-ec3d033a16a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "w": 1
                    }
                },
                "link": "assembly_jig_link",
                "collision_mesh_uuid": "environment-node.collision-mesh.-py-ec3c869416a311ec82af00155d967c6c",
                "passive": true
            },
            {
                "type": "node.machine.",
                "name": "Blade Conveyor Machine",
                "uuid": "blade-conveyor-machine-uuid",
                "editable": true,
                "deleteable": true,
                "description": "",
                "inputs": {},
                "outputs": {
                    "thing-type.-py-ec3cb4d416a311ec82af00155d967c6c": {
                        "region_uuid": "pose.region.cube-region.-py-ec3ce4f416a311ec82af00155d967c6c",
                        "quantity": 1,
                        "placeholder_uuids": [
                            "placeholder.-py-ec3ccf6416a311ec82af00155d967c6c"
                        ]
                    }
                },
                "process_time": 5,
                "mesh_id": "package://evd_ros_tasks/description/meshes/visual/conveyor.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-ec3d05f616a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "blade_conveyor_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-ec3d068c16a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0
                    },
                    "orientation": {
                        "type": "node.orientation.",
                        "name": "",
                        "uuid": "orientation.-py-ec3d07a416a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "w": 1
                    }
                },
                "link": "blade_conveyor_link",
                "collision_mesh_uuid": "environment-node.collision-mesh.-py-ec3c8b3016a311ec82af00155d967c6c",
                "passive": false
            },
            {
                "type": "node.machine.",
                "name": "Blade Feeder Machine",
                "uuid": "blade-feeder-machine-uuid",
                "editable": true,
                "deleteable": true,
                "description": "",
                "inputs": {
                    "thing-type.-py-ec3cb4d416a311ec82af00155d967c6c": {
                        "region_uuid": "pose.region.cube-region.-py-ec3ce4f416a311ec82af00155d967c6c",
                        "quantity": 1
                    }
                },
                "outputs": {
                    "thing-type.-py-ec3cb60016a311ec82af00155d967c6c": {
                        "region_uuid": "pose.region.cube-region.-py-ec3ce4f416a311ec82af00155d967c6c",
                        "quantity": 1,
                        "placeholder_uuids": [
                            "placeholder.-py-ec3cdaa416a311ec82af00155d967c6c"
                        ]
                    }
                },
                "process_time": 0,
                "mesh_id": "package://evd_ros_tasks/description/meshes/visual/blade_feeder.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-ec3d0a6016a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "blade_feeder_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-ec3d0aec16a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0
                    },
                    "orientation": {
                        "type": "node.orientation.",
                        "name": "",
                        "uuid": "orientation.-py-ec3d0c0e16a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "w": 1
                    }
                },
                "link": "blade_feeder_link",
                "collision_mesh_uuid": "environment-node.collision-mesh.-py-ec3c8fc216a311ec82af00155d967c6c",
                "passive": false
            },
            {
                "type": "node.machine.",
                "name": "Blade Conveyor Machine",
                "uuid": "knife-conveyor-machine-uuid",
                "editable": true,
                "deleteable": true,
                "description": "",
                "inputs": {
                    "thing-type.-py-ec3cb68216a311ec82af00155d967c6c": {
                        "region_uuid": "pose.region.cube-region.-py-ec3ce7f616a311ec82af00155d967c6c",
                        "quantity": 1
                    }
                },
                "outputs": {},
                "process_time": 5,
                "mesh_id": "package://evd_ros_tasks/description/meshes/visual/conveyor.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-ec3d132a16a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "knife_conveyor_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-ec3d13b616a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0
                    },
                    "orientation": {
                        "type": "node.orientation.",
                        "name": "",
                        "uuid": "orientation.-py-ec3d14ce16a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "w": 1
                    }
                },
                "link": "knife_conveyor_link",
                "collision_mesh_uuid": "environment-node.collision-mesh.-py-ec3c94ea16a311ec82af00155d967c6c",
                "passive": false
            },
            {
                "type": "node.machine.",
                "name": "Knife Feeder Machine",
                "uuid": "knife-feeder-machine-uuid",
                "editable": true,
                "deleteable": true,
                "description": "",
                "inputs": {
                    "thing-type.-py-ec3cb70416a311ec82af00155d967c6c": {
                        "region_uuid": "pose.region.cube-region.-py-ec3ce7f616a311ec82af00155d967c6c",
                        "quality": 1
                    }
                },
                "outputs": {
                    "thing-type.-py-ec3cb68216a311ec82af00155d967c6c": {
                        "region_uuid": "pose.region.cube-region.-py-ec3ce7f616a311ec82af00155d967c6c",
                        "quality": 1,
                        "placeholder_uuids": [
                            "placeholder.-py-ec3cd6ee16a311ec82af00155d967c6c"
                        ]
                    }
                },
                "process_time": 0,
                "mesh_id": "package://evd_ros_tasks/description/meshes/visual/knife_feeder.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-ec3d0ec016a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "knife_feeder_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-ec3d0f4216a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0
                    },
                    "orientation": {
                        "type": "node.orientation.",
                        "name": "",
                        "uuid": "orientation.-py-ec3d107816a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "w": 1
                    }
                },
                "link": "knife_feeder_link",
                "collision_mesh_uuid": "environment-node.collision-mesh.-py-ec3c998616a311ec82af00155d967c6c",
                "passive": true
            }
        ],
        "things": [],
        "waypoints": [],
        "trajectories": [],
        "thing_types": [
            {
                "type": "node.thing-type.",
                "name": "Left Handle",
                "uuid": "thing-type.-py-ec3cb38016a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/markers/left_handle.stl",
                "is_safe": true,
                "weight": 0
            },
            {
                "type": "node.thing-type.",
                "name": "Right Handle",
                "uuid": "thing-type.-py-ec3cb43e16a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/markers/right_handle.stl",
                "is_safe": true,
                "weight": 0
            },
            {
                "type": "node.thing-type.",
                "name": "Blade",
                "uuid": "thing-type.-py-ec3cb4d416a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/markers/blade.stl",
                "is_safe": false,
                "weight": 0
            },
            {
                "type": "node.thing-type.",
                "name": "Transport Jig",
                "uuid": "thing-type.-py-ec3cb56016a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/markers/tranport_jig.stl",
                "is_safe": true,
                "weight": 0
            },
            {
                "type": "node.thing-type.",
                "name": "Blade With Transport Jig",
                "uuid": "thing-type.-py-ec3cb60016a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/markers/blade_with_transport_jig.stl",
                "is_safe": true,
                "weight": 0
            },
            {
                "type": "node.thing-type.",
                "name": "Knife",
                "uuid": "thing-type.-py-ec3cb68216a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/markers/knife.stl",
                "is_safe": false,
                "weight": 0
            },
            {
                "type": "node.thing-type.",
                "name": "Knife With Transport Jig",
                "uuid": "thing-type.-py-ec3cb70416a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/markers/knife_with_transport_jig.stl",
                "is_safe": false,
                "weight": 0
            }
        ],
        "regions": [
            {
                "type": "node.pose.region.cube-region.",
                "name": "",
                "uuid": "pose.region.cube-region.-py-ec3cdd2416a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "link": "3d_printer_link",
                "center_position": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-ec3cdb8a16a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "x": 0,
                    "y": 0,
                    "z": 0
                },
                "center_orientation": {
                    "type": "node.orientation.",
                    "name": "",
                    "uuid": "orientation.-py-ec3cdc2016a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "x": 0,
                    "y": 0,
                    "z": 0,
                    "w": 1
                },
                "free_orientation": true,
                "uncertainty_orientation_limit": null,
                "uncertainty_orientation_alt_target": null,
                "uncertainty_x": 0.01,
                "uncertainty_y": 0.01,
                "uncertainty_z": 0.01
            },
            {
                "type": "node.pose.region.cube-region.",
                "name": "",
                "uuid": "pose.region.cube-region.-py-ec3ce1de16a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "link": "3d_printer_link",
                "center_position": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-ec3cdf4a16a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "x": 0,
                    "y": 0,
                    "z": 0
                },
                "center_orientation": {
                    "type": "node.orientation.",
                    "name": "",
                    "uuid": "orientation.-py-ec3ce11616a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "x": 0,
                    "y": 0,
                    "z": 0,
                    "w": 1
                },
                "free_orientation": true,
                "uncertainty_orientation_limit": null,
                "uncertainty_orientation_alt_target": null,
                "uncertainty_x": 0.01,
                "uncertainty_y": 0.01,
                "uncertainty_z": 0.01
            },
            {
                "type": "node.pose.region.cube-region.",
                "name": "",
                "uuid": "pose.region.cube-region.-py-ec3ce4f416a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "link": "blade_feeder_link",
                "center_position": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-ec3ce3aa16a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "x": 0,
                    "y": 0,
                    "z": 0
                },
                "center_orientation": {
                    "type": "node.orientation.",
                    "name": "",
                    "uuid": "orientation.-py-ec3ce44a16a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "x": 0,
                    "y": 0,
                    "z": 0,
                    "w": 1
                },
                "free_orientation": true,
                "uncertainty_orientation_limit": null,
                "uncertainty_orientation_alt_target": null,
                "uncertainty_x": 0.01,
                "uncertainty_y": 0.01,
                "uncertainty_z": 0.01
            },
            {
                "type": "node.pose.region.cube-region.",
                "name": "",
                "uuid": "pose.region.cube-region.-py-ec3ce7f616a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "link": "knife_feeder_link",
                "center_position": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-ec3ce69816a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "x": 0,
                    "y": 0,
                    "z": 0
                },
                "center_orientation": {
                    "type": "node.orientation.",
                    "name": "",
                    "uuid": "orientation.-py-ec3ce73816a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "x": 0,
                    "y": 0,
                    "z": 0,
                    "w": 1
                },
                "free_orientation": true,
                "uncertainty_orientation_limit": null,
                "uncertainty_orientation_alt_target": null,
                "uncertainty_x": 0.01,
                "uncertainty_y": 0.01,
                "uncertainty_z": 0.01
            },
            {
                "type": "node.pose.region.cube-region.",
                "name": "",
                "uuid": "pose.region.cube-region.-py-ec3ceb0c16a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "link": "assembly_jig_link",
                "center_position": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-ec3ce9ae16a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "x": 0,
                    "y": 0,
                    "z": 0
                },
                "center_orientation": {
                    "type": "node.orientation.",
                    "name": "",
                    "uuid": "orientation.-py-ec3cea5816a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "x": 0,
                    "y": 0,
                    "z": 0,
                    "w": 1
                },
                "free_orientation": true,
                "uncertainty_orientation_limit": null,
                "uncertainty_orientation_alt_target": null,
                "uncertainty_x": 0.01,
                "uncertainty_y": 0.01,
                "uncertainty_z": 0.01
            }
        ],
        "grade_types": [],
        "placeholders": [
            {
                "type": "node.placeholder.",
                "name": "",
                "uuid": "placeholder.-py-ec3cc0be16a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "pending_node": {
                    "type": "node.pose.thing.",
                    "name": "Left Handle Thing",
                    "uuid": "pose.thing.-py-ec3cbc6816a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "link": "app",
                    "position": "<pending>",
                    "orientation": "<pending>",
                    "thing_type_uuid": "thing-type.-py-ec3cb38016a311ec82af00155d967c6c"
                },
                "pending_fields": [
                    "position",
                    "orientation"
                ]
            },
            {
                "type": "node.placeholder.",
                "name": "",
                "uuid": "placeholder.-py-ec3cc4d816a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "pending_node": {
                    "type": "node.pose.thing.",
                    "name": "Right Handle Thing",
                    "uuid": "pose.thing.-py-ec3cc19a16a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "link": "app",
                    "position": "<pending>",
                    "orientation": "<pending>",
                    "thing_type_uuid": "thing-type.-py-ec3cb43e16a311ec82af00155d967c6c"
                },
                "pending_fields": [
                    "position",
                    "orientation"
                ]
            },
            {
                "type": "node.placeholder.",
                "name": "",
                "uuid": "placeholder.-py-ec3ccafa16a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "pending_node": {
                    "type": "node.pose.thing.",
                    "name": "Transport Jig Placeholder",
                    "uuid": "pose.thing.-py-ec3cc6c216a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "link": "app",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-ec3cc57816a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": true,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0
                    },
                    "orientation": {
                        "type": "node.orientation.",
                        "name": "",
                        "uuid": "orientation.-py-ec3cc60e16a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": true,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "w": 1
                    },
                    "thing_type_uuid": "thing-type.-py-ec3cb56016a311ec82af00155d967c6c"
                },
                "pending_fields": []
            },
            {
                "type": "node.placeholder.",
                "name": "",
                "uuid": "placeholder.-py-ec3ccf6416a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "pending_node": {
                    "type": "node.pose.thing.",
                    "name": "Blade Thing",
                    "uuid": "pose.thing.-py-ec3ccbf416a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "link": "app",
                    "position": "<pending>",
                    "orientation": "<pending>",
                    "thing_type_uuid": "thing-type.-py-ec3cb4d416a311ec82af00155d967c6c"
                },
                "pending_fields": [
                    "position",
                    "orientation"
                ]
            },
            {
                "type": "node.placeholder.",
                "name": "",
                "uuid": "placeholder.-py-ec3cd33816a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "pending_node": {
                    "type": "node.pose.thing.",
                    "name": "Blade with Transport Jig Thing",
                    "uuid": "pose.thing.-py-ec3cd01816a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "link": "app",
                    "position": "<pending>",
                    "orientation": "<pending>",
                    "thing_type_uuid": "thing-type.-py-ec3cb60016a311ec82af00155d967c6c"
                },
                "pending_fields": [
                    "position",
                    "orientation"
                ]
            },
            {
                "type": "node.placeholder.",
                "name": "",
                "uuid": "placeholder.-py-ec3cd6ee16a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "pending_node": {
                    "type": "node.pose.thing.",
                    "name": "Knife Thing",
                    "uuid": "pose.thing.-py-ec3cd3d816a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "link": "app",
                    "position": "<pending>",
                    "orientation": "<pending>",
                    "thing_type_uuid": "thing-type.-py-ec3cb68216a311ec82af00155d967c6c"
                },
                "pending_fields": [
                    "position",
                    "orientation"
                ]
            },
            {
                "type": "node.placeholder.",
                "name": "",
                "uuid": "placeholder.-py-ec3cdaa416a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "pending_node": {
                    "type": "node.pose.thing.",
                    "name": "Knife with Transport Jig Thing",
                    "uuid": "pose.thing.-py-ec3cd79816a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "link": "app",
                    "position": "<pending>",
                    "orientation": "<pending>",
                    "thing_type_uuid": "thing-type.-py-ec3cb70416a311ec82af00155d967c6c"
                },
                "pending_fields": [
                    "position",
                    "orientation"
                ]
            }
        ],
        "reach_sphere": {
            "type": "node.environment-node.reach-sphere.",
            "name": "",
            "uuid": "environment-node.reach-sphere.-py-ec3c34dc16a311ec82af00155d967c6c",
            "editable": true,
            "deleteable": true,
            "description": "",
            "radius": 0.8,
            "offset": {
                "type": "node.position.",
                "name": "",
                "uuid": "position.-py-ec3c337e16a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "x": 0,
                "y": 0,
                "z": 0.15
            },
            "link": "simulated_base_link"
        },
        "pinch_points": [
            {
                "type": "node.environment-node.pinch-point.",
                "name": "",
                "uuid": "environment-node.pinch-point.-py-ec3c607e16a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "axis": "x",
                "offset": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-ec3c5f0c16a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "x": 0,
                    "y": 0,
                    "z": -0.05
                },
                "link": "simulated_shoulder_link",
                "radius": 0.075,
                "length": 0.2
            },
            {
                "type": "node.environment-node.pinch-point.",
                "name": "",
                "uuid": "environment-node.pinch-point.-py-ec3c62f416a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "axis": "x",
                "offset": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-ec3c623616a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "x": 0,
                    "y": 0,
                    "z": 0.075
                },
                "link": "simulated_upper_arm_link",
                "radius": 0.075,
                "length": 0.2
            },
            {
                "type": "node.environment-node.pinch-point.",
                "name": "",
                "uuid": "environment-node.pinch-point.-py-ec3c64fc16a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "axis": "x",
                "offset": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-ec3c644816a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "x": 0,
                    "y": 0,
                    "z": 0.075
                },
                "link": "simulated_forearm_link",
                "radius": 0.075,
                "length": 0.2
            },
            {
                "type": "node.environment-node.pinch-point.",
                "name": "",
                "uuid": "environment-node.pinch-point.-py-ec3c66fa16a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "axis": "x",
                "offset": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-ec3c665a16a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "x": 0,
                    "y": 0,
                    "z": -0.05
                },
                "link": "simulated_wrist_1_link",
                "radius": 0.06,
                "length": 0.17
            },
            {
                "type": "node.environment-node.pinch-point.",
                "name": "",
                "uuid": "environment-node.pinch-point.-py-ec3c68d016a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "axis": "x",
                "offset": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-ec3c682616a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "x": 0,
                    "y": 0,
                    "z": 0.1
                },
                "link": "simulated_wrist_3_link",
                "radius": 0.1,
                "length": 0.16
            }
        ],
        "collision_meshes": [
            {
                "type": "node.environment-node.collision-mesh.",
                "name": "3D Printer Collision Mesh",
                "uuid": "environment-node.collision-mesh.-py-ec3c6fc416a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/meshes/collision/3d_printer.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-ec3c70aa16a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "3d_printer_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-ec3c715e16a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0
                    },
                    "orientation": {
                        "type": "node.orientation.",
                        "name": "",
                        "uuid": "orientation.-py-ec3c72bc16a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "w": 1
                    }
                },
                "link": "3d_printer_link"
            },
            {
                "type": "node.environment-node.collision-mesh.",
                "name": "Pedestal Collision Mesh",
                "uuid": "environment-node.collision-mesh.-py-ec3c7c9e16a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/meshes/collision/pedestal.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-ec3c7d7016a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "ur3e_pedestal_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-ec3c7e1016a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0
                    },
                    "orientation": {
                        "type": "node.orientation.",
                        "name": "",
                        "uuid": "orientation.-py-ec3c7f5016a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "w": 1
                    }
                },
                "link": "ur3e_pedestal_link"
            },
            {
                "type": "node.environment-node.collision-mesh.",
                "name": "Table Collision Mesh",
                "uuid": "environment-node.collision-mesh.-py-ec3c81da16a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/meshes/collision/table.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-ec3c827016a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "table_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-ec3c82fc16a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0
                    },
                    "orientation": {
                        "type": "node.orientation.",
                        "name": "",
                        "uuid": "orientation.-py-ec3c842816a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "w": 1
                    }
                },
                "link": "table_link"
            },
            {
                "type": "node.environment-node.collision-mesh.",
                "name": "Assembly Jig Collision Mesh",
                "uuid": "environment-node.collision-mesh.-py-ec3c869416a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/meshes/collision/assembly_jig.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-ec3c872016a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "assembly_jig_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-ec3c87ac16a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0
                    },
                    "orientation": {
                        "type": "node.orientation.",
                        "name": "",
                        "uuid": "orientation.-py-ec3c88c416a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "w": 1
                    }
                },
                "link": "assembly_jig_link"
            },
            {
                "type": "node.environment-node.collision-mesh.",
                "name": "Blade Conveyor Collision Mesh",
                "uuid": "environment-node.collision-mesh.-py-ec3c8b3016a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/meshes/collision/conveyor.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-ec3c8bbc16a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "blade_conveyor_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-ec3c8c3416a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0
                    },
                    "orientation": {
                        "type": "node.orientation.",
                        "name": "",
                        "uuid": "orientation.-py-ec3c8d4c16a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "w": 1
                    }
                },
                "link": "blade_conveyor_link"
            },
            {
                "type": "node.environment-node.collision-mesh.",
                "name": "Blade Feeder Collision Mesh",
                "uuid": "environment-node.collision-mesh.-py-ec3c8fc216a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/meshes/collision/blade_feeder.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-ec3c904e16a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "blade_feeder_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-ec3c90d016a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0
                    },
                    "orientation": {
                        "type": "node.orientation.",
                        "name": "",
                        "uuid": "orientation.-py-ec3c91e816a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "w": 1
                    }
                },
                "link": "blade_feeder_link"
            },
            {
                "type": "node.environment-node.collision-mesh.",
                "name": "Knife Conveyor Collision Mesh",
                "uuid": "environment-node.collision-mesh.-py-ec3c94ea16a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/meshes/collision/conveyor.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-ec3c958016a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "knife_conveyor_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-ec3c960c16a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0
                    },
                    "orientation": {
                        "type": "node.orientation.",
                        "name": "",
                        "uuid": "orientation.-py-ec3c972416a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "w": 1
                    }
                },
                "link": "knife_conveyor_link"
            },
            {
                "type": "node.environment-node.collision-mesh.",
                "name": "Knife Feeder Collision Mesh",
                "uuid": "environment-node.collision-mesh.-py-ec3c998616a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/meshes/collision/knife_feeder.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-ec3c9a1216a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "knife_feeder_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-ec3c9a9416a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0
                    },
                    "orientation": {
                        "type": "node.orientation.",
                        "name": "",
                        "uuid": "orientation.-py-ec3c9bc016a311ec82af00155d967c6c",
                        "editable": true,
                        "deleteable": false,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "w": 1
                    }
                },
                "link": "knife_feeder_link"
            }
        ],
        "occupancy_zones": [
            {
                "type": "node.environment-node.occupancy-zone.",
                "name": "Robot Occupancy Zone",
                "uuid": "environment-node.occupancy-zone.-py-ec3cadf416a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "occupancy_type": "robot",
                "position_x": 0,
                "position_z": 0,
                "scale_x": 1.6,
                "scale_z": 1.2,
                "height": -0.8
            },
            {
                "type": "node.environment-node.occupancy-zone.",
                "name": "Human Workspace Occupancy Zone",
                "uuid": "environment-node.occupancy-zone.-py-ec3caf2a16a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "occupancy_type": "human",
                "position_x": 0,
                "position_z": 1,
                "scale_x": 2,
                "scale_z": 1,
                "height": -0.8
            },
            {
                "type": "node.environment-node.occupancy-zone.",
                "name": "Human Corridor Occupancy Zone",
                "uuid": "environment-node.occupancy-zone.-py-ec3caff216a311ec82af00155d967c6c",
                "editable": true,
                "deleteable": true,
                "description": "",
                "occupancy_type": "human",
                "position_x": 0,
                "position_z": -1,
                "scale_x": 2,
                "scale_z": 1,
                "height": -0.8
            }
        ]
    },
    "skills": [
        {
            "type": "node.primitive.hierarchical.skill.",
            "name": "Machine Blocking Process",
            "uuid": "primitive.hierarchical.skill.-py-ec3c0d7c16a311ec82af00155d967c6c",
            "editable": true,
            "deleteable": false,
            "description": "",
            "parameters": {},
            "primitives": [
                {
                    "type": "node.primitive.machine-primitive.machine-start.",
                    "name": "",
                    "uuid": "primitive.machine-primitive.machine-start.-py-ec3c0ad416a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "machine_uuid": "skill-argument-variable-py-ec3c09ee16a311ec82af00155d967c6c"
                    }
                },
                {
                    "type": "node.primitive.machine-primitive.machine-wait.",
                    "name": "",
                    "uuid": "primitive.machine-primitive.machine-wait.-py-ec3c0bc416a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "machine_uuid": "skill-argument-variable-py-ec3c09ee16a311ec82af00155d967c6c"
                    }
                },
                {
                    "type": "node.primitive.machine-primitive.machine-stop.",
                    "name": "",
                    "uuid": "primitive.machine-primitive.machine-stop.-py-ec3c0c8c16a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "machine_uuid": "skill-argument-variable-py-ec3c09ee16a311ec82af00155d967c6c"
                    }
                }
            ],
            "arguments": []
        },
        {
            "type": "node.primitive.hierarchical.skill.",
            "name": "Simple Pick And Place",
            "uuid": "primitive.hierarchical.skill.-py-ec3c183a16a311ec82af00155d967c6c",
            "editable": true,
            "deleteable": false,
            "description": "",
            "parameters": {},
            "primitives": [
                {
                    "type": "node.primitive.move-trajectory.",
                    "name": "",
                    "uuid": "primitive.move-trajectory.-py-ec3c152e16a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "manual_safety": false,
                        "trajectory_uuid": "skill-argument-variable-py-ec3c127c16a311ec82af00155d967c6c"
                    }
                },
                {
                    "type": "node.primitive.gripper.",
                    "name": "",
                    "uuid": "primitive.gripper.-py-ec3c161416a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "thing_uuid": "skill-argument-variable-py-ec3c147016a311ec82af00155d967c6c",
                        "position": 100,
                        "effort": 0,
                        "speed": 0,
                        "semantic": "grasping"
                    }
                },
                {
                    "type": "node.primitive.move-trajectory.",
                    "name": "",
                    "uuid": "primitive.move-trajectory.-py-ec3c16e616a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "manual_safety": false,
                        "trajectory_uuid": "skill-argument-variable-py-ec3c138a16a311ec82af00155d967c6c"
                    }
                },
                {
                    "type": "node.primitive.gripper.",
                    "name": "",
                    "uuid": "primitive.gripper.-py-ec3c178616a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "thing_uuid": "skill-argument-variable-py-ec3c147016a311ec82af00155d967c6c",
                        "position": 0,
                        "effort": 0,
                        "speed": 0,
                        "semantic": "releasing"
                    }
                }
            ],
            "arguments": []
        },
        {
            "type": "node.primitive.hierarchical.skill.",
            "name": "Open Gripper",
            "uuid": "primitive.hierarchical.skill.-py-ec3c218616a311ec82af00155d967c6c",
            "editable": true,
            "deleteable": false,
            "description": "",
            "parameters": {},
            "primitives": [
                {
                    "type": "node.primitive.gripper.",
                    "name": "",
                    "uuid": "primitive.gripper.-py-ec3c20b416a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "thing_uuid": "skill-argument-variable-py-ec3c201416a311ec82af00155d967c6c",
                        "position": 0,
                        "effort": 0,
                        "speed": 0,
                        "semantic": "releasing"
                    }
                }
            ],
            "arguments": []
        },
        {
            "type": "node.primitive.hierarchical.skill.",
            "name": "Close Gripper",
            "uuid": "primitive.hierarchical.skill.-py-ec3c24b016a311ec82af00155d967c6c",
            "editable": true,
            "deleteable": false,
            "description": "",
            "parameters": {},
            "primitives": [
                {
                    "type": "node.primitive.gripper.",
                    "name": "",
                    "uuid": "primitive.gripper.-py-ec3c23fc16a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "thing_uuid": "skill-argument-variable-py-ec3c237016a311ec82af00155d967c6c",
                        "position": 100,
                        "effort": 0,
                        "speed": 0,
                        "semantic": "grasping"
                    }
                }
            ],
            "arguments": []
        },
        {
            "type": "node.primitive.hierarchical.skill.",
            "name": "Initialize",
            "uuid": "primitive.hierarchical.skill.-py-ec3c28ca16a311ec82af00155d967c6c",
            "editable": true,
            "deleteable": false,
            "description": "",
            "parameters": {},
            "primitives": [
                {
                    "type": "node.primitive.gripper.",
                    "name": "",
                    "uuid": "primitive.gripper.-py-ec3c272616a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "thing_uuid": null,
                        "position": 0,
                        "effort": 0,
                        "speed": 0,
                        "semantic": "grasping"
                    }
                },
                {
                    "type": "node.primitive.move-unplanned.",
                    "name": "",
                    "uuid": "primitive.move-unplanned.-py-ec3c280216a311ec82af00155d967c6c",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "velocity": 0,
                        "move_type": "joint",
                        "location_uuid": "skill-argument-variable-py-ec3c268616a311ec82af00155d967c6c",
                        "manual_safety": true
                    }
                }
            ],
            "arguments": []
        }
    ]
}

export default KNIFE_TASK