const KNIFE_TASK = {
    "name": "Knife Assembly",
    "uuid": "primitive.hierarchical.program.-py-98888c021c1911ecbe2600155d1a70a2",
    "type": "node.primitive.hierarchical.program.",
    "description": "",
    "transform": {
        "x": 30,
        "y": 10
    },
    "environment": {
        "grade_types": [],
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
        "locations": [
            {
                "type": "node.pose.waypoint.location.",
                "uuid": "node.pose.waypoint.location-js-c540bea6-a0a8-40c2-8fcc-cb6ae772697c",
                "name": "Initial Location",
                "deleteable": true,
                "editable": true,
                "description": "Some descriptor string (optional)",
                "link": "",
                "joints": {
                    "type": "node.joints.",
                    "uuid": "node.joints.-js-559f8a5d-982e-49e6-9fbc-239f179bffb6",
                    "name": "",
                    "deleteable": false,
                    "editable": true,
                    "description": "",
                    "joint_positions": null,
                    "joint_names": null,
                    "reachable": false,
                    "length": 5
                },
                "position": {
                    "type": "node.position.",
                    "uuid": "node.position.-js-34ee9653-943a-4ea3-8c9e-3be407df805d",
                    "name": "",
                    "deleteable": false,
                    "editable": true,
                    "description": "",
                    "x": 0,
                    "y": 0.11273895681396362,
                    "z": 0.35624279886072235
                },
                "orientation": {
                    "type": "node.orientation.",
                    "uuid": "node.orientation.-js-e89bd4c4-f157-467f-bb08-f705e7f1031b",
                    "name": "",
                    "deleteable": false,
                    "editable": true,
                    "description": "",
                    "x": 0,
                    "y": 0,
                    "z": 0.6763577452496884,
                    "w": 0.7365732824646558
                }
            },
            {
                "type": "node.pose.waypoint.location.",
                "uuid": "node.pose.waypoint.location-js-b7daabbd-6e24-4b8f-9c8e-5ea22d727ad0",
                "name": "New Location",
                "deleteable": true,
                "editable": true,
                "description": "Some descriptor string (optional)",
                "link": "",
                "joints": {
                    "type": "node.joints.",
                    "uuid": "node.joints.-js-ace8e311-f37d-4e00-9516-d0b8b53e52b7",
                    "name": "",
                    "deleteable": false,
                    "editable": true,
                    "description": "",
                    "joint_positions": null,
                    "joint_names": null,
                    "reachable": false,
                    "length": 5
                },
                "position": {
                    "type": "node.position.",
                    "uuid": "node.position.-js-1f9cbda0-1c52-442b-b309-06c543a29ffb",
                    "name": "",
                    "deleteable": false,
                    "editable": true,
                    "description": "",
                    "x": 0.3491669265198674,
                    "y": -0.46708392645887964,
                    "z": 0.2896457164587347
                },
                "orientation": {
                    "type": "node.orientation.",
                    "uuid": "node.orientation.-js-8d671b25-662c-496c-a682-8c205c35d791",
                    "name": "",
                    "deleteable": false,
                    "editable": true,
                    "description": "",
                    "x": 0,
                    "y": 0,
                    "z": 0,
                    "w": 1
                }
            }
        ],
        "waypoints": [],
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
                    "thing-type.-py-98892bd01c1911ecbe2600155d1a70a2": [
                        {
                            "region_uuid": "pose.region.cube-region.-py-98898a621c1911ecbe2600155d1a70a2",
                            "quantity": 1
                        }
                    ],
                    "thing-type.-py-98892a7c1c1911ecbe2600155d1a70a2": [
                        {
                            "region_uuid": "pose.region.cube-region.-py-98898a621c1911ecbe2600155d1a70a2",
                            "quantity": 1
                        }
                    ],
                    "thing-type.-py-98892b3a1c1911ecbe2600155d1a70a2": [
                        {
                            "region_uuid": "pose.region.cube-region.-py-98898a621c1911ecbe2600155d1a70a2",
                            "quantity": 1
                        }
                    ]
                },
                "outputs": {
                    "thing-type.-py-98892d7e1c1911ecbe2600155d1a70a2": [
                        {
                            "region_uuid": "pose.region.cube-region.-py-98898a621c1911ecbe2600155d1a70a2",
                            "quantity": 1,
                            "placeholder_uuids": [
                                "placeholder.-py-9889539e1c1911ecbe2600155d1a70a2"
                            ]
                        }
                    ]
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
                "name": "Assembly Jig W/ Transport",
                "uuid": "assembly-jig-machine-uuid_safe",
                "editable": true,
                "deleteable": true,
                "description": "",
                "inputs": {
                    "thing-type.-py-98892ce81c1911ecbe2600155d1a70a2": [
                        {
                            "region_uuid": "pose.region.cube-region.-py-98898a621c1911ecbe2600155d1a70a2",
                            "quantity": 1
                        }
                    ],
                    "thing-type.-py-98892a7c1c1911ecbe2600155d1a70a2": [
                        {
                            "region_uuid": "pose.region.cube-region.-py-98898a621c1911ecbe2600155d1a70a2",
                            "quantity": 1
                        }
                    ],
                    "thing-type.-py-98892b3a1c1911ecbe2600155d1a70a2": [
                        {
                            "region_uuid": "pose.region.cube-region.-py-98898a621c1911ecbe2600155d1a70a2",
                            "quantity": 1
                        }
                    ]
                },
                "outputs": {
                    "thing-type.-py-98892e001c1911ecbe2600155d1a70a2": [
                        {
                            "region_uuid": "pose.region.cube-region.-py-98898a621c1911ecbe2600155d1a70a2",
                            "quantity": 1,
                            "placeholder_uuids": [
                                "placeholder.-py-988965641c1911ecbe2600155d1a70a2"
                            ]
                        }
                    ]
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
                    "thing-type.-py-98892bd01c1911ecbe2600155d1a70a2": [
                        {
                            "region_uuid": "pose.region.cube-region.-py-98897e321c1911ecbe2600155d1a70a2",
                            "quantity": 1,
                            "placeholder_uuids": [
                                "placeholder.-py-9889435e1c1911ecbe2600155d1a70a2"
                            ]
                        }
                    ]
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
                    "thing-type.-py-98892bd01c1911ecbe2600155d1a70a2": [
                        {
                            "region_uuid": "pose.region.cube-region.-py-98897e321c1911ecbe2600155d1a70a2",
                            "quantity": 1
                        }
                    ]
                },
                "outputs": {
                    "thing-type.-py-98892ce81c1911ecbe2600155d1a70a2": [
                        {
                            "region_uuid": "pose.region.cube-region.-py-98897e321c1911ecbe2600155d1a70a2",
                            "quantity": 1,
                            "placeholder_uuids": [
                                "placeholder.-py-988965641c1911ecbe2600155d1a70a2"
                            ]
                        }
                    ]
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
                    "thing-type.-py-98892d7e1c1911ecbe2600155d1a70a2": [
                        {
                            "region_uuid": "pose.region.cube-region.-py-988982ec1c1911ecbe2600155d1a70a2",
                            "quantity": 1
                        }
                    ]
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
                    "thing-type.-py-98892e001c1911ecbe2600155d1a70a2": [
                        {
                            "region_uuid": "pose.region.cube-region.-py-988982ec1c1911ecbe2600155d1a70a2",
                            "quantity": 1
                        }
                    ]
                },
                "outputs": {
                    "thing-type.-py-98892d7e1c1911ecbe2600155d1a70a2": [
                        {
                            "region_uuid": "pose.region.cube-region.-py-988982ec1c1911ecbe2600155d1a70a2",
                            "quantity": 1,
                            "placeholder_uuids": [
                                "placeholder.-py-9889539e1c1911ecbe2600155d1a70a2"
                            ]
                        }
                    ]
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
        "trajectories": [
            {
                "uuid": "node.trajectory.-js-0ca9faa4-0d76-48c6-9752-397a16edc1de",
                "type": "node.trajectory.",
                "name": "New Trajectory",
                "deleteable": true,
                "editable": true,
                "description": "A movement by the robot from one location to another.",
                "start_location_uuid": "node.pose.waypoint.location-js-c540bea6-a0a8-40c2-8fcc-cb6ae772697c",
                "end_location_uuid": "node.pose.waypoint.location-js-b7daabbd-6e24-4b8f-9c8e-5ea22d727ad0",
                "waypoint_uuids": [],
                "trace": null,
                "velocity": 0.5,
                "move_type": "ee_ik"
            }
        ],
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
        "placeholders": [
            {
                "type": "node.placeholder.",
                "name": "",
                "uuid": "placeholder.-py-9889377e1c1911ecbe2600155d1a70a2",
                "editable": true,
                "deleteable": true,
                "description": "",
                "pending_node": {
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
                "pending_node": {
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
                "pending_node": {
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
                "pending_node": {
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
                "pending_node": {
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
                "pending_node": {
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
                "pending_node": {
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
        "regions": [
            {
                "type": "node.pose.region.cube-region.",
                "name": "Left Handle Spawn",
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
                "name": "Right Handle Spawn",
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
                "name": "Blade Receiver",
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
                "name": "Blade Dispatch",
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
                "name": "Assembly Region",
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
        ]
    },
    "primitives": [
        {
            "type": "node.primitive.skill-call.",
            "uuid": "skill-call-js-f2fdde0a-6a74-4af7-8e44-5ab25d0a62dc",
            "name": "Execute Skill: Initialize",
            "editable": true,
            "deleteable": true,
            "description": "",
            "parameters": {
                "skill_uuid": "primitive.hierarchical.skill.-py-9888b04c1c1911ecbe2600155d1a70a2",
                "skill-arg-js-03d2b7eb-78ba-459d-a698-1728c371a94b": "node.pose.waypoint.location-js-c540bea6-a0a8-40c2-8fcc-cb6ae772697c"
            },
            "parentData": {
                "type": "program",
                "uuid": "primitive.hierarchical.program.-py-98888c021c1911ecbe2600155d1a70a2"
            }
        },
        {
            "uuid": "node.primitive.move-trajectory.-js-dca0cc0d-0c55-4d6d-ba90-748f6f80b51d",
            "type": "node.primitive.move-trajectory.",
            "name": "Move Trajectory",
            "description": "Move the robot arm along a trajectory",
            "editable": true,
            "deleteable": true,
            "parameters": {
                "manual_safety": false,
                "trajectory_uuid": "node.trajectory.-js-0ca9faa4-0d76-48c6-9752-397a16edc1de"
            }
        }
    ],
    "skills": [
        {
            "type": "node.primitive.hierarchical.skill.",
            "name": "Machine Blocking Process",
            "uuid": "primitive.hierarchical.skill.-py-988896fc1c1911ecbe2600155d1a70a2",
            "editable": true,
            "deleteable": false,
            "description": "",
            "parameters": {},
            "arguments": [
                {
                    "uuid": "skill-arg-js-6774c040-91f0-4b43-8374-e3e735cd6882",
                    "name": "Process Machine",
                    "description": "",
                    "is_list": false,
                    "parameter_type": "node.machine.",
                    "type": "node.skill-argument.",
                    "editable": true,
                    "deleteable": true
                }
            ],
            "transform": {
                "x": 580,
                "y": 10
            },
            "primitives": [
                {
                    "type": "node.primitive.machine-primitive.machine-start.",
                    "name": "Machine Start",
                    "uuid": "primitive.machine-primitive.machine-start.-py-9888945e1c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "machine_uuid": "skill-arg-js-6774c040-91f0-4b43-8374-e3e735cd6882"
                    },
                    "parentData": {
                        "type": "skill",
                        "uuid": "primitive.hierarchical.skill.-py-988896fc1c1911ecbe2600155d1a70a2"
                    }
                },
                {
                    "type": "node.primitive.machine-primitive.machine-wait.",
                    "name": "Machine Wait",
                    "uuid": "primitive.machine-primitive.machine-wait.-py-9888954e1c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "machine_uuid": "skill-arg-js-6774c040-91f0-4b43-8374-e3e735cd6882"
                    },
                    "parentData": {
                        "type": "skill",
                        "uuid": "primitive.hierarchical.skill.-py-988896fc1c1911ecbe2600155d1a70a2"
                    }
                },
                {
                    "type": "node.primitive.machine-primitive.machine-stop.",
                    "name": "Machine Stop",
                    "uuid": "primitive.machine-primitive.machine-stop.-py-988896161c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "machine_uuid": "skill-arg-js-6774c040-91f0-4b43-8374-e3e735cd6882"
                    },
                    "parentData": {
                        "type": "skill",
                        "uuid": "primitive.hierarchical.skill.-py-988896fc1c1911ecbe2600155d1a70a2"
                    }
                }
            ]
        },
        {
            "type": "node.primitive.hierarchical.skill.",
            "name": "Simple Pick And Place",
            "uuid": "primitive.hierarchical.skill.-py-9888a1d81c1911ecbe2600155d1a70a2",
            "editable": false,
            "deleteable": false,
            "description": "",
            "parameters": {},
            "arguments": [
                {
                    "uuid": "skill-arg-js-9fd596bf-051d-46cd-88d6-e1599cca058f",
                    "name": "Moved Object",
                    "description": "",
                    "is_list": false,
                    "parameter_type": "node.pose.thing.",
                    "type": "node.skill-argument.",
                    "editable": false,
                    "deleteable": false
                },
                {
                    "uuid": "skill-arg-js-70cb74c7-e959-47de-a099-be264eccc8c9",
                    "name": "Pick Trajectory",
                    "description": "",
                    "is_list": false,
                    "parameter_type": "node.trajectory.",
                    "type": "node.skill-argument.",
                    "editable": false,
                    "deleteable": false
                },
                {
                    "uuid": "skill-arg-js-7ba7b50c-76ae-497e-a47a-4ca4b44c7024",
                    "name": "Place Trajectory",
                    "description": "",
                    "is_list": false,
                    "parameter_type": "node.trajectory.",
                    "type": "node.skill-argument.",
                    "editable": false,
                    "deleteable": false
                }
            ],
            "transform": {
                "x": 1010,
                "y": 10
            },
            "primitives": [
                {
                    "type": "node.primitive.move-trajectory.",
                    "name": "Move Trajectory",
                    "uuid": "primitive.move-trajectory.-py-98889e901c1911ecbe2600155d1a70a2",
                    "editable": false,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "manual_safety": false,
                        "trajectory_uuid": "skill-arg-js-70cb74c7-e959-47de-a099-be264eccc8c9"
                    },
                    "parentData": {
                        "type": "skill",
                        "uuid": "primitive.hierarchical.skill.-py-9888a1d81c1911ecbe2600155d1a70a2"
                    }
                },
                {
                    "type": "node.primitive.gripper.",
                    "name": "Grasp",
                    "uuid": "primitive.gripper.-py-98889f941c1911ecbe2600155d1a70a2",
                    "editable": false,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "thing_uuid": "skill-arg-js-9fd596bf-051d-46cd-88d6-e1599cca058f",
                        "position": 100,
                        "effort": 0,
                        "speed": 0,
                        "semantic": "grasping"
                    },
                    "parentData": {
                        "type": "skill",
                        "uuid": "primitive.hierarchical.skill.-py-9888a1d81c1911ecbe2600155d1a70a2"
                    }
                },
                {
                    "type": "node.primitive.move-trajectory.",
                    "name": "Move Trajectory",
                    "uuid": "primitive.move-trajectory.-py-9888a0661c1911ecbe2600155d1a70a2",
                    "editable": false,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "manual_safety": false,
                        "trajectory_uuid": "skill-arg-js-7ba7b50c-76ae-497e-a47a-4ca4b44c7024"
                    },
                    "parentData": {
                        "type": "skill",
                        "uuid": "primitive.hierarchical.skill.-py-9888a1d81c1911ecbe2600155d1a70a2"
                    }
                },
                {
                    "type": "node.primitive.gripper.",
                    "name": "Release",
                    "uuid": "primitive.gripper.-py-9888a1061c1911ecbe2600155d1a70a2",
                    "editable": false,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "thing_uuid": "skill-arg-js-9fd596bf-051d-46cd-88d6-e1599cca058f",
                        "position": 0,
                        "effort": 0,
                        "speed": 0,
                        "semantic": "releasing"
                    },
                    "parentData": {
                        "type": "skill",
                        "uuid": "primitive.hierarchical.skill.-py-9888a1d81c1911ecbe2600155d1a70a2"
                    }
                }
            ]
        },
        {
            "type": "node.primitive.hierarchical.skill.",
            "name": "Open Gripper",
            "uuid": "primitive.hierarchical.skill.-py-9888a7be1c1911ecbe2600155d1a70a2",
            "editable": true,
            "deleteable": false,
            "description": "",
            "parameters": {},
            "arguments": [
                {
                    "uuid": "skill-arg-js-291e7d29-f1fa-4b7d-b634-9c37837bbb35",
                    "name": "Grasped Object",
                    "description": "",
                    "is_list": false,
                    "parameter_type": "node.pose.thing.",
                    "type": "node.skill-argument.",
                    "editable": true,
                    "deleteable": true
                }
            ],
            "transform": {
                "x": 1440,
                "y": 10
            },
            "primitives": [
                {
                    "type": "node.primitive.gripper.",
                    "name": "Release",
                    "uuid": "primitive.gripper.-py-9888a7001c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "thing_uuid": "skill-arg-js-291e7d29-f1fa-4b7d-b634-9c37837bbb35",
                        "position": 0,
                        "effort": 0,
                        "speed": 0,
                        "semantic": "releasing"
                    },
                    "parentData": {
                        "type": "skill",
                        "uuid": "primitive.hierarchical.skill.-py-9888a7be1c1911ecbe2600155d1a70a2"
                    }
                }
            ]
        },
        {
            "type": "node.primitive.hierarchical.skill.",
            "name": "Close Gripper",
            "uuid": "primitive.hierarchical.skill.-py-9888aaca1c1911ecbe2600155d1a70a2",
            "editable": true,
            "deleteable": false,
            "description": "",
            "parameters": {},
            "arguments": [
                {
                    "uuid": "skill-arg-js-ca360c3a-1cf5-4700-9efd-eb5fb3f9e96c",
                    "name": "Grasped Object",
                    "description": "",
                    "is_list": false,
                    "parameter_type": "node.pose.thing.",
                    "type": "node.skill-argument.",
                    "editable": true,
                    "deleteable": true
                }
            ],
            "transform": {
                "x": 1870,
                "y": 10
            },
            "primitives": [
                {
                    "type": "node.primitive.gripper.",
                    "name": "Grasp",
                    "uuid": "primitive.gripper.-py-9888aa161c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "thing_uuid": "skill-arg-js-ca360c3a-1cf5-4700-9efd-eb5fb3f9e96c",
                        "position": 100,
                        "effort": 0,
                        "speed": 0,
                        "semantic": "grasping"
                    },
                    "parentData": {
                        "type": "skill",
                        "uuid": "primitive.hierarchical.skill.-py-9888aaca1c1911ecbe2600155d1a70a2"
                    }
                }
            ]
        },
        {
            "type": "node.primitive.hierarchical.skill.",
            "name": "Initialize",
            "uuid": "primitive.hierarchical.skill.-py-9888b04c1c1911ecbe2600155d1a70a2",
            "editable": true,
            "deleteable": false,
            "description": "",
            "parameters": {},
            "arguments": [
                {
                    "uuid": "skill-arg-js-03d2b7eb-78ba-459d-a698-1728c371a94b",
                    "name": "Starting Location",
                    "description": "",
                    "is_list": false,
                    "parameter_type": "node.pose.waypoint.location.",
                    "type": "node.skill-argument.",
                    "editable": true,
                    "deleteable": true
                }
            ],
            "transform": {
                "x": 2300,
                "y": 10
            },
            "primitives": [
                {
                    "type": "node.primitive.move-unplanned.",
                    "name": "Move Unplanned",
                    "uuid": "primitive.move-unplanned.-py-9888af7a1c1911ecbe2600155d1a70a2",
                    "editable": true,
                    "deleteable": false,
                    "description": "",
                    "parameters": {
                        "velocity": 0,
                        "move_type": "joint",
                        "location_uuid": "skill-arg-js-03d2b7eb-78ba-459d-a698-1728c371a94b",
                        "manual_safety": true
                    },
                    "parentData": {
                        "type": "skill",
                        "uuid": "primitive.hierarchical.skill.-py-9888b04c1c1911ecbe2600155d1a70a2"
                    }
                },
                {
                    "uuid": "node.primitive.machine-primitive.machine-initialize.-js-76093ce3-46b9-4ed5-b1ec-5195500881bd",
                    "type": "node.primitive.machine-primitive.machine-initialize.",
                    "name": "Machine Initialize",
                    "description": "Initialize a machine",
                    "editable": true,
                    "deleteable": true,
                    "parameters": {
                        "machine_uuid": "3d-printer-machine-uuid"
                    },
                    "parentData": {
                        "type": "skill",
                        "uuid": "primitive.hierarchical.skill.-py-9888b04c1c1911ecbe2600155d1a70a2"
                    }
                },
                {
                    "uuid": "node.primitive.machine-primitive.machine-initialize.-js-aeff7e71-74c0-44dd-b8d1-9b81b63515bc",
                    "type": "node.primitive.machine-primitive.machine-initialize.",
                    "name": "Machine Initialize",
                    "description": "Initialize a machine",
                    "editable": true,
                    "deleteable": true,
                    "parameters": {
                        "machine_uuid": "assembly-jig-machine-uuid_unsafe"
                    },
                    "parentData": {
                        "type": "skill",
                        "uuid": "primitive.hierarchical.skill.-py-9888b04c1c1911ecbe2600155d1a70a2"
                    }
                },
                {
                    "uuid": "node.primitive.machine-primitive.machine-initialize.-js-4da845af-9c78-4a7a-8fc1-8022c1cf1235",
                    "type": "node.primitive.machine-primitive.machine-initialize.",
                    "name": "Machine Initialize",
                    "description": "Initialize a machine",
                    "editable": true,
                    "deleteable": true,
                    "parameters": {
                        "machine_uuid": "assembly-jig-machine-uuid_safe"
                    },
                    "parentData": {
                        "type": "skill",
                        "uuid": "primitive.hierarchical.skill.-py-9888b04c1c1911ecbe2600155d1a70a2"
                    }
                },
                {
                    "uuid": "node.primitive.machine-primitive.machine-initialize.-js-56b4ae05-9e71-4ee9-ba09-4791eb0aad4c",
                    "type": "node.primitive.machine-primitive.machine-initialize.",
                    "name": "Machine Initialize",
                    "description": "Initialize a machine",
                    "editable": true,
                    "deleteable": true,
                    "parameters": {
                        "machine_uuid": "blade-conveyor-machine-uuid"
                    },
                    "parentData": {
                        "type": "skill",
                        "uuid": "primitive.hierarchical.skill.-py-9888b04c1c1911ecbe2600155d1a70a2"
                    }
                },
                {
                    "uuid": "node.primitive.machine-primitive.machine-initialize.-js-b37d4d20-fad6-42ee-8d1a-bd12f7d49db8",
                    "type": "node.primitive.machine-primitive.machine-initialize.",
                    "name": "Machine Initialize",
                    "description": "Initialize a machine",
                    "editable": true,
                    "deleteable": true,
                    "parameters": {
                        "machine_uuid": "blade-feeder-machine-uuid"
                    },
                    "parentData": {
                        "type": "skill",
                        "uuid": "primitive.hierarchical.skill.-py-9888b04c1c1911ecbe2600155d1a70a2"
                    }
                },
                {
                    "uuid": "node.primitive.machine-primitive.machine-initialize.-js-a5962c6d-2521-40c8-a665-3a8322b26519",
                    "type": "node.primitive.machine-primitive.machine-initialize.",
                    "name": "Machine Initialize",
                    "description": "Initialize a machine",
                    "editable": true,
                    "deleteable": true,
                    "parameters": {
                        "machine_uuid": "knife-conveyor-machine-uuid"
                    },
                    "parentData": {
                        "type": "skill",
                        "uuid": "primitive.hierarchical.skill.-py-9888b04c1c1911ecbe2600155d1a70a2"
                    }
                },
                {
                    "uuid": "node.primitive.machine-primitive.machine-initialize.-js-96d621cb-f504-4c06-bc83-60cf399ff0e4",
                    "type": "node.primitive.machine-primitive.machine-initialize.",
                    "name": "Machine Initialize",
                    "description": "Initialize a machine",
                    "editable": true,
                    "deleteable": true,
                    "parameters": {
                        "machine_uuid": "knife-feeder-machine-uuid"
                    },
                    "parentData": {
                        "type": "skill",
                        "uuid": "primitive.hierarchical.skill.-py-9888b04c1c1911ecbe2600155d1a70a2"
                    }
                }
            ]
        }
    ]
}

export default KNIFE_TASK