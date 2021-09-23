const KNIFE_TASK = {
    "type": "node.primitive.hierarchical.program.",
    "name": "",
    "uuid": "primitive.hierarchical.program.-py-98888c021c1911ecbe2600155d1a70a2",
    "editable": true,
    "deleteable": true,
    "description": "",
    "parameters": {},
    "primitives": [],
    "environment": {
        "type": "node.context.environment.",
        "name": "",
        "uuid": "context.environment.-py-988889aa1c1911ecbe2600155d1a70a2",
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
                    "thing-type.-py-98892a7c1c1911ecbe2600155d1a70a2": [
                        {
                            "region_uuid": "pose.region.cube-region.-py-98896bcc1c1911ecbe2600155d1a70a2",
                            "quantity": 1,
                            "placeholder_uuids": [
                                "placeholder.-py-9889377e1c1911ecbe2600155d1a70a2"
                            ]
                        }
                    ],
                    "thing-type.-py-98892b3a1c1911ecbe2600155d1a70a2": [
                        {
                            "region_uuid": "pose.region.cube-region.-py-988976441c1911ecbe2600155d1a70a2",
                            "quantity": 1,
                            "placeholder_uuids": [
                                "placeholder.-py-98893ba21c1911ecbe2600155d1a70a2"
                            ]
                        }
                    ]
                },
                "process_time": 5,
                "mesh_id": "package://evd_ros_tasks/description/meshes/visual/3d_printer.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-98899fd41c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "3d_printer_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-9889a0f61c1911ecbe2600155d1a70a2",
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
                        "uuid": "orientation.-py-9889a2861c1911ecbe2600155d1a70a2",
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
                "collision_mesh_uuid": "environment-node.collision-mesh.-py-9888ecd81c1911ecbe2600155d1a70a2",
                "passive": false
            },
            {
                "type": "node.machine.",
                "name": "Assembly Jig Machine",
                "uuid": "assembly-jig-machine-uuid_unsafe",
                "editable": true,
                "deleteable": true,
                "description": "",
                "inputs": {
                    "thing-type.-py-98892bd01c1911ecbe2600155d1a70a2": {
                        "region_uuid": "pose.region.cube-region.-py-98898a621c1911ecbe2600155d1a70a2",
                        "quantity": 1
                    },
                    "thing-type.-py-98892a7c1c1911ecbe2600155d1a70a2": {
                        "region_uuid": "pose.region.cube-region.-py-98898a621c1911ecbe2600155d1a70a2",
                        "quantity": 1
                    },
                    "thing-type.-py-98892b3a1c1911ecbe2600155d1a70a2": {
                        "region_uuid": "pose.region.cube-region.-py-98898a621c1911ecbe2600155d1a70a2",
                        "quantity": 1
                    }
                },
                "outputs": {
                    "thing-type.-py-98892d7e1c1911ecbe2600155d1a70a2": {
                        "region_uuid": "pose.region.cube-region.-py-98898a621c1911ecbe2600155d1a70a2",
                        "quantity": 1,
                        "placeholder_uuids": [
                            "placeholder.-py-9889539e1c1911ecbe2600155d1a70a2"
                        ]
                    }
                },
                "process_time": 0,
                "mesh_id": "package://evd_ros_tasks/description/meshes/visual/assembly_jig.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-9889aa921c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "assembly_jig_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-9889ab781c1911ecbe2600155d1a70a2",
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
                        "uuid": "orientation.-py-9889af4c1c1911ecbe2600155d1a70a2",
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
                "collision_mesh_uuid": "environment-node.collision-mesh.-py-9888ff521c1911ecbe2600155d1a70a2",
                "passive": true
            },
            {
                "type": "node.machine.",
                "name": "Assembly Jig Machine",
                "uuid": "assembly-jig-machine-uuid_safe",
                "editable": true,
                "deleteable": true,
                "description": "",
                "inputs": {
                    "thing-type.-py-98892ce81c1911ecbe2600155d1a70a2": {
                        "region_uuid": "pose.region.cube-region.-py-98898a621c1911ecbe2600155d1a70a2",
                        "quantity": 1
                    },
                    "thing-type.-py-98892a7c1c1911ecbe2600155d1a70a2": {
                        "region_uuid": "pose.region.cube-region.-py-98898a621c1911ecbe2600155d1a70a2",
                        "quantity": 1
                    },
                    "thing-type.-py-98892b3a1c1911ecbe2600155d1a70a2": {
                        "region_uuid": "pose.region.cube-region.-py-98898a621c1911ecbe2600155d1a70a2",
                        "quantity": 1
                    }
                },
                "outputs": {
                    "thing-type.-py-98892e001c1911ecbe2600155d1a70a2": {
                        "region_uuid": "pose.region.cube-region.-py-98898a621c1911ecbe2600155d1a70a2",
                        "quantity": 1,
                        "placeholder_uuids": [
                            "placeholder.-py-988965641c1911ecbe2600155d1a70a2"
                        ]
                    }
                },
                "process_time": 0,
                "mesh_id": "package://evd_ros_tasks/description/meshes/visual/assembly_jig.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-9889b3b61c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "assembly_jig_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-9889b4421c1911ecbe2600155d1a70a2",
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
                        "uuid": "orientation.-py-9889b5641c1911ecbe2600155d1a70a2",
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
                "collision_mesh_uuid": "environment-node.collision-mesh.-py-9888ff521c1911ecbe2600155d1a70a2",
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
                    "thing-type.-py-98892bd01c1911ecbe2600155d1a70a2": {
                        "region_uuid": "pose.region.cube-region.-py-98897e321c1911ecbe2600155d1a70a2",
                        "quantity": 1,
                        "placeholder_uuids": [
                            "placeholder.-py-9889435e1c1911ecbe2600155d1a70a2"
                        ]
                    }
                },
                "process_time": 5,
                "mesh_id": "package://evd_ros_tasks/description/meshes/visual/conveyor.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-9889b8201c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "blade_conveyor_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-9889b8ac1c1911ecbe2600155d1a70a2",
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
                        "uuid": "orientation.-py-9889b9c41c1911ecbe2600155d1a70a2",
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
                "collision_mesh_uuid": "environment-node.collision-mesh.-py-988903ee1c1911ecbe2600155d1a70a2",
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
                    "thing-type.-py-98892bd01c1911ecbe2600155d1a70a2": {
                        "region_uuid": "pose.region.cube-region.-py-98897e321c1911ecbe2600155d1a70a2",
                        "quantity": 1
                    }
                },
                "outputs": {
                    "thing-type.-py-98892ce81c1911ecbe2600155d1a70a2": {
                        "region_uuid": "pose.region.cube-region.-py-98897e321c1911ecbe2600155d1a70a2",
                        "quantity": 1,
                        "placeholder_uuids": [
                            "placeholder.-py-988965641c1911ecbe2600155d1a70a2"
                        ]
                    }
                },
                "process_time": 0,
                "mesh_id": "package://evd_ros_tasks/description/meshes/visual/blade_feeder.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-9889bc9e1c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "blade_feeder_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-9889bd201c1911ecbe2600155d1a70a2",
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
                        "uuid": "orientation.-py-9889be381c1911ecbe2600155d1a70a2",
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
                "collision_mesh_uuid": "environment-node.collision-mesh.-py-988908621c1911ecbe2600155d1a70a2",
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
                    "thing-type.-py-98892d7e1c1911ecbe2600155d1a70a2": {
                        "region_uuid": "pose.region.cube-region.-py-988982ec1c1911ecbe2600155d1a70a2",
                        "quantity": 1
                    }
                },
                "outputs": {},
                "process_time": 5,
                "mesh_id": "package://evd_ros_tasks/description/meshes/visual/conveyor.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-9889c5541c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "knife_conveyor_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-9889c6bc1c1911ecbe2600155d1a70a2",
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
                        "uuid": "orientation.-py-9889c84c1c1911ecbe2600155d1a70a2",
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
                "collision_mesh_uuid": "environment-node.collision-mesh.-py-98890ccc1c1911ecbe2600155d1a70a2",
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
                    "thing-type.-py-98892e001c1911ecbe2600155d1a70a2": {
                        "region_uuid": "pose.region.cube-region.-py-988982ec1c1911ecbe2600155d1a70a2",
                        "quality": 1
                    }
                },
                "outputs": {
                    "thing-type.-py-98892d7e1c1911ecbe2600155d1a70a2": {
                        "region_uuid": "pose.region.cube-region.-py-988982ec1c1911ecbe2600155d1a70a2",
                        "quality": 1,
                        "placeholder_uuids": [
                            "placeholder.-py-9889539e1c1911ecbe2600155d1a70a2"
                        ]
                    }
                },
                "process_time": 0,
                "mesh_id": "package://evd_ros_tasks/description/meshes/visual/knife_feeder.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-9889c0ea1c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "knife_feeder_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-9889c1761c1911ecbe2600155d1a70a2",
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
                        "uuid": "orientation.-py-9889c2a21c1911ecbe2600155d1a70a2",
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
                "collision_mesh_uuid": "environment-node.collision-mesh.-py-9889112c1c1911ecbe2600155d1a70a2",
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
                "uuid": "thing-type.-py-98892a7c1c1911ecbe2600155d1a70a2",
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
                "uuid": "thing-type.-py-98892b3a1c1911ecbe2600155d1a70a2",
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
                "uuid": "thing-type.-py-98892bd01c1911ecbe2600155d1a70a2",
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
                "uuid": "thing-type.-py-98892c5c1c1911ecbe2600155d1a70a2",
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
                "uuid": "thing-type.-py-98892ce81c1911ecbe2600155d1a70a2",
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
                "uuid": "thing-type.-py-98892d7e1c1911ecbe2600155d1a70a2",
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
                "uuid": "thing-type.-py-98892e001c1911ecbe2600155d1a70a2",
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
                "uuid": "pose.region.cube-region.-py-98896bcc1c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "link": "3d_printer_link",
                "center_position": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-988968521c1911ecbe2600155d1a70a2",
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
                    "uuid": "orientation.-py-988969b01c1911ecbe2600155d1a70a2",
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
                "uuid": "pose.region.cube-region.-py-988976441c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "link": "3d_printer_link",
                "center_position": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-98896f0a1c1911ecbe2600155d1a70a2",
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
                    "uuid": "orientation.-py-988973ec1c1911ecbe2600155d1a70a2",
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
                "uuid": "pose.region.cube-region.-py-98897e321c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "link": "blade_feeder_link",
                "center_position": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-988979f01c1911ecbe2600155d1a70a2",
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
                    "uuid": "orientation.-py-98897b941c1911ecbe2600155d1a70a2",
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
                "uuid": "pose.region.cube-region.-py-988982ec1c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "link": "knife_feeder_link",
                "center_position": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-988980ee1c1911ecbe2600155d1a70a2",
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
                    "uuid": "orientation.-py-9889818e1c1911ecbe2600155d1a70a2",
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
                "uuid": "pose.region.cube-region.-py-98898a621c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "link": "assembly_jig_link",
                "center_position": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-988986d41c1911ecbe2600155d1a70a2",
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
                    "uuid": "orientation.-py-988988be1c1911ecbe2600155d1a70a2",
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
                "uuid": "placeholder.-py-9889377e1c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "pending_node_dct": {
                    "type": "node.pose.thing.",
                    "name": "Left Handle Thing",
                    "uuid": "pose.thing.-py-988933781c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "link": "app",
                    "position": "<pending>",
                    "orientation": "<pending>",
                    "thing_type_uuid": "thing-type.-py-98892a7c1c1911ecbe2600155d1a70a2"
                },
                "pending_fields": [
                    "position",
                    "orientation"
                ]
            },
            {
                "type": "node.placeholder.",
                "name": "",
                "uuid": "placeholder.-py-98893ba21c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "pending_node_dct": {
                    "type": "node.pose.thing.",
                    "name": "Right Handle Thing",
                    "uuid": "pose.thing.-py-988938501c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "link": "app",
                    "position": "<pending>",
                    "orientation": "<pending>",
                    "thing_type_uuid": "thing-type.-py-98892b3a1c1911ecbe2600155d1a70a2"
                },
                "pending_fields": [
                    "position",
                    "orientation"
                ]
            },
            {
                "type": "node.placeholder.",
                "name": "",
                "uuid": "placeholder.-py-98893f761c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "pending_node_dct": {
                    "type": "node.pose.thing.",
                    "name": "Transport Jig Placeholder",
                    "uuid": "pose.thing.-py-98893d821c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "link": "app",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-98893c381c1911ecbe2600155d1a70a2",
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
                        "uuid": "orientation.-py-98893cce1c1911ecbe2600155d1a70a2",
                        "editable": true,
                        "deleteable": true,
                        "description": "",
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "w": 1
                    },
                    "thing_type_uuid": "thing-type.-py-98892c5c1c1911ecbe2600155d1a70a2"
                },
                "pending_fields": []
            },
            {
                "type": "node.placeholder.",
                "name": "",
                "uuid": "placeholder.-py-9889435e1c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "pending_node_dct": {
                    "type": "node.pose.thing.",
                    "name": "Blade Thing",
                    "uuid": "pose.thing.-py-988940161c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "link": "app",
                    "position": "<pending>",
                    "orientation": "<pending>",
                    "thing_type_uuid": "thing-type.-py-98892bd01c1911ecbe2600155d1a70a2"
                },
                "pending_fields": [
                    "position",
                    "orientation"
                ]
            },
            {
                "type": "node.placeholder.",
                "name": "",
                "uuid": "placeholder.-py-988947001c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "pending_node_dct": {
                    "type": "node.pose.thing.",
                    "name": "Blade with Transport Jig Thing",
                    "uuid": "pose.thing.-py-988943fe1c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "link": "app",
                    "position": "<pending>",
                    "orientation": "<pending>",
                    "thing_type_uuid": "thing-type.-py-98892ce81c1911ecbe2600155d1a70a2"
                },
                "pending_fields": [
                    "position",
                    "orientation"
                ]
            },
            {
                "type": "node.placeholder.",
                "name": "",
                "uuid": "placeholder.-py-9889539e1c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "pending_node_dct": {
                    "type": "node.pose.thing.",
                    "name": "Knife Thing",
                    "uuid": "pose.thing.-py-988947be1c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "link": "app",
                    "position": "<pending>",
                    "orientation": "<pending>",
                    "thing_type_uuid": "thing-type.-py-98892d7e1c1911ecbe2600155d1a70a2"
                },
                "pending_fields": [
                    "position",
                    "orientation"
                ]
            },
            {
                "type": "node.placeholder.",
                "name": "",
                "uuid": "placeholder.-py-988965641c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "pending_node_dct": {
                    "type": "node.pose.thing.",
                    "name": "Knife with Transport Jig Thing",
                    "uuid": "pose.thing.-py-988957721c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": true,
                    "description": "",
                    "link": "app",
                    "position": "<pending>",
                    "orientation": "<pending>",
                    "thing_type_uuid": "thing-type.-py-98892e001c1911ecbe2600155d1a70a2"
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
            "uuid": "environment-node.reach-sphere.-py-9888bb821c1911ecbe2600155d1a70a2",
            "editable": true,
            "deleteable": true,
            "description": "",
            "radius": 0.8,
            "offset": {
                "type": "node.position.",
                "name": "",
                "uuid": "position.-py-9888ba061c1911ecbe2600155d1a70a2",
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
                "uuid": "environment-node.pinch-point.-py-9888ddc41c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "axis": "x",
                "offset": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-9888dc7a1c1911ecbe2600155d1a70a2",
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
                "uuid": "environment-node.pinch-point.-py-9888e01c1c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "axis": "x",
                "offset": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-9888df5e1c1911ecbe2600155d1a70a2",
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
                "uuid": "environment-node.pinch-point.-py-9888e21a1c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "axis": "x",
                "offset": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-9888e1701c1911ecbe2600155d1a70a2",
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
                "uuid": "environment-node.pinch-point.-py-9888e40e1c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "axis": "x",
                "offset": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-9888e36e1c1911ecbe2600155d1a70a2",
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
                "uuid": "environment-node.pinch-point.-py-9888e5e41c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "axis": "x",
                "offset": {
                    "type": "node.position.",
                    "name": "",
                    "uuid": "position.-py-9888e53a1c1911ecbe2600155d1a70a2",
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
                "uuid": "environment-node.collision-mesh.-py-9888ecd81c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/meshes/collision/3d_printer.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-9888edb41c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "3d_printer_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-9888ee5e1c1911ecbe2600155d1a70a2",
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
                        "uuid": "orientation.-py-9888efb21c1911ecbe2600155d1a70a2",
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
                "uuid": "environment-node.collision-mesh.-py-9888f49e1c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/meshes/collision/pedestal.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-9888f53e1c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "ur3e_pedestal_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-9888f5ca1c1911ecbe2600155d1a70a2",
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
                        "uuid": "orientation.-py-9888f6ec1c1911ecbe2600155d1a70a2",
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
                "uuid": "environment-node.collision-mesh.-py-9888f9621c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/meshes/collision/table.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-9888fae81c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "table_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-9888fb9c1c1911ecbe2600155d1a70a2",
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
                        "uuid": "orientation.-py-9888fce61c1911ecbe2600155d1a70a2",
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
                "uuid": "environment-node.collision-mesh.-py-9888ff521c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/meshes/collision/assembly_jig.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-9888ffde1c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "assembly_jig_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-988900601c1911ecbe2600155d1a70a2",
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
                        "uuid": "orientation.-py-988901781c1911ecbe2600155d1a70a2",
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
                "uuid": "environment-node.collision-mesh.-py-988903ee1c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/meshes/collision/conveyor.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-9889047a1c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "blade_conveyor_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-988904fc1c1911ecbe2600155d1a70a2",
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
                        "uuid": "orientation.-py-988906141c1911ecbe2600155d1a70a2",
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
                "uuid": "environment-node.collision-mesh.-py-988908621c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/meshes/collision/blade_feeder.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-988908f81c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "blade_feeder_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-988909701c1911ecbe2600155d1a70a2",
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
                        "uuid": "orientation.-py-98890a7e1c1911ecbe2600155d1a70a2",
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
                "uuid": "environment-node.collision-mesh.-py-98890ccc1c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/meshes/collision/conveyor.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-98890d581c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "knife_conveyor_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-98890dd01c1911ecbe2600155d1a70a2",
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
                        "uuid": "orientation.-py-98890ee81c1911ecbe2600155d1a70a2",
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
                "uuid": "environment-node.collision-mesh.-py-9889112c1c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "mesh_id": "package://evd_ros_tasks/description/meshes/collision/knife_feeder.stl",
                "pose_offset": {
                    "type": "node.pose.",
                    "name": "",
                    "uuid": "pose.-py-988911b81c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "link": "knife_feeder_link",
                    "position": {
                        "type": "node.position.",
                        "name": "",
                        "uuid": "position.-py-988912301c1911ecbe2600155d1a70a2",
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
                        "uuid": "orientation.-py-9889133e1c1911ecbe2600155d1a70a2",
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
                "uuid": "environment-node.occupancy-zone.-py-988924c81c1911ecbe2600155d1a70a2",
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
                "uuid": "environment-node.occupancy-zone.-py-988926121c1911ecbe2600155d1a70a2",
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
                "uuid": "environment-node.occupancy-zone.-py-988926d01c1911ecbe2600155d1a70a2",
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
            "uuid": "primitive.hierarchical.skill.-py-988896fc1c1911ecbe2600155d1a70a2",
            "editable": true,
            "deleteable": false,
            "description": "",
            "parameters": {},
            "primitives": [
                {
                    "type": "node.primitive.machine-primitive.machine-start.",
                    "name": "",
                    "uuid": "primitive.machine-primitive.machine-start.-py-9888945e1c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "machine_uuid": "skill-argument-variable-py-988893821c1911ecbe2600155d1a70a2"
                    }
                },
                {
                    "type": "node.primitive.machine-primitive.machine-wait.",
                    "name": "",
                    "uuid": "primitive.machine-primitive.machine-wait.-py-9888954e1c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "machine_uuid": "skill-argument-variable-py-988893821c1911ecbe2600155d1a70a2"
                    }
                },
                {
                    "type": "node.primitive.machine-primitive.machine-stop.",
                    "name": "",
                    "uuid": "primitive.machine-primitive.machine-stop.-py-988896161c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "machine_uuid": "skill-argument-variable-py-988893821c1911ecbe2600155d1a70a2"
                    }
                }
            ],
            "arguments": []
        },
        {
            "type": "node.primitive.hierarchical.skill.",
            "name": "Simple Pick And Place",
            "uuid": "primitive.hierarchical.skill.-py-9888a1d81c1911ecbe2600155d1a70a2",
            "editable": true,
            "deleteable": false,
            "description": "",
            "parameters": {},
            "primitives": [
                {
                    "type": "node.primitive.move-trajectory.",
                    "name": "",
                    "uuid": "primitive.move-trajectory.-py-98889e901c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "manual_safety": false,
                        "trajectory_uuid": "skill-argument-variable-py-98889c101c1911ecbe2600155d1a70a2"
                    }
                },
                {
                    "type": "node.primitive.gripper.",
                    "name": "",
                    "uuid": "primitive.gripper.-py-98889f941c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "thing_uuid": "skill-argument-variable-py-98889df01c1911ecbe2600155d1a70a2",
                        "position": 100,
                        "effort": 0,
                        "speed": 0,
                        "semantic": "grasping"
                    }
                },
                {
                    "type": "node.primitive.move-trajectory.",
                    "name": "",
                    "uuid": "primitive.move-trajectory.-py-9888a0661c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "manual_safety": false,
                        "trajectory_uuid": "skill-argument-variable-py-98889cf61c1911ecbe2600155d1a70a2"
                    }
                },
                {
                    "type": "node.primitive.gripper.",
                    "name": "",
                    "uuid": "primitive.gripper.-py-9888a1061c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "thing_uuid": "skill-argument-variable-py-98889df01c1911ecbe2600155d1a70a2",
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
            "uuid": "primitive.hierarchical.skill.-py-9888a7be1c1911ecbe2600155d1a70a2",
            "editable": true,
            "deleteable": false,
            "description": "",
            "parameters": {},
            "primitives": [
                {
                    "type": "node.primitive.gripper.",
                    "name": "",
                    "uuid": "primitive.gripper.-py-9888a7001c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "thing_uuid": "skill-argument-variable-py-9888a6741c1911ecbe2600155d1a70a2",
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
            "uuid": "primitive.hierarchical.skill.-py-9888aaca1c1911ecbe2600155d1a70a2",
            "editable": true,
            "deleteable": false,
            "description": "",
            "parameters": {},
            "primitives": [
                {
                    "type": "node.primitive.gripper.",
                    "name": "",
                    "uuid": "primitive.gripper.-py-9888aa161c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "thing_uuid": "skill-argument-variable-py-9888a9941c1911ecbe2600155d1a70a2",
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
            "uuid": "primitive.hierarchical.skill.-py-9888b04c1c1911ecbe2600155d1a70a2",
            "editable": true,
            "deleteable": false,
            "description": "",
            "parameters": {},
            "primitives": [
                {
                    "type": "node.primitive.gripper.",
                    "name": "",
                    "uuid": "primitive.gripper.-py-9888ae9e1c1911ecbe2600155d1a70a2",
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
                    "uuid": "primitive.move-unplanned.-py-9888af7a1c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "velocity": 0,
                        "move_type": "joint",
                        "location_uuid": "skill-argument-variable-py-9888ade01c1911ecbe2600155d1a70a2",
                        "manual_safety": true
                    }
                }
            ],
            "arguments": []
        }
    ]
}

export default KNIFE_TASK