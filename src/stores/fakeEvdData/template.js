
// So in addition to the node, dot delimited scheme there are some
// default types that are provided to further complicate type lookup :)
// - "<all>"
// - "<all-primitives>"
// - "<all-regions>"
// - "<all-conditions>"
// - "<all-skills>"
// - "<location-or-waypoint>"
// - "<string>"
// - "<number>"
// - "<boolean>"
// - "<enum>"               (if supplied then an enum_values field should be expected in the template with the list)
// - "<arbitrary-obj>"      (if supplied then further parsing will require a priori / external knowledge of data)
// - "<parameters>"         (if supplied then the node is a primitive that has a list of parameters (template will have a parameters field))

/***************************************************************** 
* Node
*****************************************************************/

const node_template = {
    type: 'node.',
    name: 'Node', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        //empty
    ]
};

/***************************************************************** 
* Type Declaration
* - ThingType
* - GradeType
* - SkillArgument
*****************************************************************/

const thingType_template = {
    type: 'node.thing-type.',
    name: 'Thing Type', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<string>',
            key: 'mesh_id',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<boolean>',
            key: 'is_safe',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'weight',
            is_uuid: false,
            is_list: false
        }
    ]
}

const gradeType_template = {
    type: 'node.grade-type.',
    name: 'Grade Type', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        //empty
    ]
}

const skillArgument_template = {
    type: 'node.skill-argument.',
    name: 'Skill Argument', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<string>',
            key: 'parameter_key',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<string>',
            key: 'temporary_value',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<string>',
            key: 'parameter_type',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<boolean>',
            key: 'is_list',
            is_uuid: false,
            is_list: false
        }
    ]
};

const placholder_template = {
    type: 'node.placeholder.',
    name: 'Placeholder', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [
        {
            type: '<arbitrary-obj>',
            key: 'pending_node',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<string>',
            key: 'pending_fields',
            is_uuid: false,
            is_list: true
        }
    ]
};

/***************************************************************** 
*  Core geometry
*****************************************************************/

const pose_template = {
    type: 'node.pose.',
    name: 'Pose', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [
        {
            type: 'node.position.',
            key: 'position',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.orientation.',
            key: 'orientation',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<string>',
            key: 'link',
            is_uuid: false,
            is_list: false
        }
    ]
};

const position_template = {
    type: 'node.position.',
    name: 'Position', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [
        {
            type: '<number>',
            key: 'x',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'y',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'z',
            is_uuid: false,
            is_list: false
        }
    ]
};

const orientation_template = {
    type: 'node.orientation.',
    name: 'Orientation', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [
        {
            type: '<number>',
            key: 'x',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'y',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'z',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'w',
            is_uuid: false,
            is_list: false
        }
    ]
};

const joints_template = {
    type: 'node.joints.',
    name: 'Joints', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [
        {
            type: '<number>',
            key: 'length',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'joint_positions',
            is_uuid: false,
            is_list: true
        },
        {
            type: '<string>',
            key: 'joint_names',
            is_uuid: false,
            is_list: true
        },
        {
            type: '<boolean>',
            key: 'reachable',
            is_uuid: false,
            is_list: false
        }
    ]
};

/***************************************************************** 
*  Waypoints and Locations
*****************************************************************/

const waypoint_template = {
    type: 'node.pose.waypoint.',
    name: 'Waypoint', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [
        {
            type: 'node.position.',
            key: 'position',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.orientation.',
            key: 'orientation',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.joints.',
            key: 'joints',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<string>',
            key: 'link',
            is_uuid: false,
            is_list: false
        }
    ]
};

const location_template = {
    type: 'node.pose.waypoint.location.',
    name: 'Location', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [
        {
            type: 'node.position.',
            key: 'position',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.orientation.',
            key: 'orientation',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.joints.',
            key: 'joints',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<string>',
            key: 'link',
            is_uuid: false,
            is_list: false
        }
    ]
};

/***************************************************************** 
*  Things
*****************************************************************/

const thing_template = {
    type: 'node.pose.thing',
    name: 'Thing', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [
        {
            type: 'node.position.',
            key: 'position',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.orientation.',
            key: 'orientation',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.thing-type.',
            key: 'thing_type_uuid',
            is_uuid: true,
            is_list: false
        },
        {
            type: '<string>',
            key: 'link',
            is_uuid: false,
            is_list: false
        }
    ]
};

/***************************************************************** 
* Regions
*****************************************************************/

const region_template = {
    type: 'node.pose.region.',
    name: 'Orientation Region', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [
        {
            type: 'node.position.',
            key: 'center_position',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.orientation.',
            key: 'center_orientation',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<boolean>',
            key: 'free_orientation',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'uncertainty_orientation_limit',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.orientation.',
            key: 'uncertainty_orientation_alt_target',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<string>',
            key: 'link',
            is_uuid: false,
            is_list: false
        }
    ]
};

const cube_region_template = {
    type: 'node.pose.region.cube-region.',
    name: 'Cube Region', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [
        {
            type: 'node.position.',
            key: 'center_position',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.orientation.',
            key: 'center_orientation',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<boolean>',
            key: 'free_orientation',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'uncertainty_orientation_limit',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.orientation.',
            key: 'uncertainty_orientation_alt_target',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'uncertainty_x',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'uncertainty_y',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'uncertainty_z',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<string>',
            key: 'link',
            is_uuid: false,
            is_list: false
        }
    ]
};

const sphere_region_template = {
    type: 'node.pose.region.sphere-region.',
    name: 'Sphere Region', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [
        {
            type: 'node.position.',
            key: 'center_position',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.orientation.',
            key: 'center_orientation',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<boolean>',
            key: 'free_orientation',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'uncertainty_orientation_limit',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.orientation.',
            key: 'uncertainty_orientation_alt_target',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'uncertainty_radius',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<string>',
            key: 'link',
            is_uuid: false,
            is_list: false
        }
    ]
};

/***************************************************************** 
* Machines
*****************************************************************/

const machine_template = {
    type: 'node.machine.',
    name: 'Machine', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<arbitrary-obj>', //dictionary of lists of dictionaries of region uuid and quantities with thing_type uuid as key
            key: 'inputs',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<arbitrary-obj>', //dictionary of lists of dictionaries of region uuid and quantities with thing_type uuid as key
            key: 'outputs',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'process_time',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<string>',
            key: 'mesh_id',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.pose.',
            key: 'pose_offset',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<string>',
            key: 'link',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.environment-node.collision-mesh.',
            key: 'collision_mesh_uuid',
            is_uuid: true,
            is_list: false
        }
    ]
}

/***************************************************************** 
* Trajectories
* - Trajectory
*   - Trace
*****************************************************************/

const trajectory_template = {
    type: 'node.trajectory.',
    name: 'Trajectory', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: 'node.pose.waypoint.location.',
            key: 'start_location_uuid',
            is_uuid: true,
            is_list: false
        },
        {
            type: 'node.pose.waypoint.location.',
            key: 'end_location_uuid',
            is_uuid: true,
            is_list: false
        },
        {
            type: 'node.pose.waypoint.',
            key: 'waypoint_uuid',
            is_uuid: true,
            is_list: true
        },
        {
            type: '<number>',
            key: 'velocity',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<enum>',
            key: 'move_type',
            is_uuid: false,
            is_list: false,
            enum_values: [
                'joints',
                'ee_ik'
            ]
        },
        {
            type: 'node.trace.',
            key: 'trace',
            is_uuid: false,
            is_list: false
        }
    ] 
}

const trace_template = {
    type: 'node.trace.',
    name: 'Trace', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // instance fields
        {
            type: '<number>',
            key: 'time_data',
            is_uuid: false,
            is_list: true
        },
        {
            type: '<string>',
            key: 'joint_names',
            is_uuid: false,
            is_list: true
        },
        {
            type: '<arbitrary-obj>', // array of number array
            key: 'joint_data',
            is_uuid: false,
            is_list: true
        },
        {
            type: '<arbitrary-obj>', // dict of array of simple poses with frame keys
            key: 'tf_data',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<arbitrary-obj>', // dict of array of numbers with grade_type uuids as keys
            key: 'grades',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<string>',
            key: 'end_effector_path',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<string>',
            key: 'joint_paths',
            is_uuid: false,
            is_list: true
        },
        {
            type: '<string>',
            key: 'tool_paths',
            is_uuid: false,
            is_list: true
        },
        {
            type: '<string>',
            key: 'component_paths',
            is_uuid: false,
            is_list: true
        },
        {
            type: '<number>',
            key: 'duration',
            is_uuid: false,
            is_list: false
        }
    ]
}

/***************************************************************** 
* Environment Objects
* - Reach Sphere
* - Collision Mesh
* - Occupancy Zone
* - Pinch Point
*****************************************************************/

const environmentNode_template = {
    type: 'node.environment-node.',
    name: 'Environment Node', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        //empty
    ]
};

const reachSphere_template = {
    type: 'node.environment-node.reach-sphere',
    name: 'Reach Sphere', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<number>',
            key: 'radius',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.position.',
            key: 'offset',
            is_uuid: false,
            is_list: false
        }
    ]
};

const collisionMesh_template = {
    type: 'node.environment-node.collision-mesh.',
    name: 'Collision Mesh', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<string>',
            key: 'mesh_id',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.pose.',
            key: 'pose_offset',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<string>',
            key: 'link',
            is_uuid: false,
            is_list: false
        }
    ]
};

const pinchPoint_template = {
    type: 'node.environment-node.reach-sphere',
    name: 'Reach Sphere', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<enum>',
            key: 'axis',
            is_uuid: false,
            is_list: false,
            enum_values: [
                'x',
                'y',
                'z'
            ]
        },
        {
            type: 'node.position.',
            key: 'offset',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<string>',
            key: 'link',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'radius',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'length',
            is_uuid: false,
            is_list: false
        }
    ]
};

const occupancyZone_template = {
    type: 'node.environment-node.occupancy-zone.',
    name: 'Occupancy Zone', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<enum>',
            key: 'occupancy_type',
            is_uuid: false,
            is_list: false,
            enum_values: [
                'human',
                'robot'
            ]
        },
        {
            type: '<number>',
            key: 'position_x',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'position_z',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'scale_x',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'scale_z',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'height',
            is_uuid: false,
            is_list: false
        }
    ]
};

/***************************************************************** 
* Context and Environment
*****************************************************************/

const context_template = {
    type: 'node.context.',
    name: 'Context', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: 'node.pose.waypoint.location.',
            key: 'locations',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'node.machine.',
            key: 'machines',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'node.pose.thing.',
            key: 'thigns',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'node.thing-type.',
            key: 'thing_types',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'node.pose.waypoint.',
            key: 'waypoints',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'node.trajectory.',
            key: 'trajectories',
            is_uuid: false,
            is_list: true
        },
        {
            type: '<all-regions>',
            key: 'regions',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'node.grade-type.',
            key: 'grade_types',
            is_uuid: false,
            is_list: true
        },
    ]
};

const environment_template = {
    type: 'node.context.environment.',
    name: 'Environment', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: 'node.pose.waypoint.location.',
            key: 'locations',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'node.machine.',
            key: 'machines',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'node.pose.thing.',
            key: 'thigns',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'node.thing-type.',
            key: 'thing_types',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'node.pose.waypoint.',
            key: 'waypoints',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'node.trajectory.',
            key: 'trajectories',
            is_uuid: false,
            is_list: true
        },
        {
            type: '<all-regions>',
            key: 'regions',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'node.grade-type.',
            key: 'grade_types',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'node.environment-node.reach-sphere.',
            key: 'reach_sphere',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.environment-node.pinch-point.',
            key: 'pinch_points',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'node.environment-node.collision-mesh.',
            key: 'collision_meshes',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'node.environment-node.occupancy-zone.',
            key: 'occupancy_zones',
            is_uuid: false,
            is_list: true
        }
    ]
};

/***************************************************************** 
* Primitives
*****************************************************************/

const primitive_template = {
    type: 'node.primitive.',
    name: 'Primitive', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        //empty
    ]
};

const machine_primitive_template = {
    type: 'node.primitive.machine-primitive.',
    name: 'Machine Primitive', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        {
            type: 'node.machine.',
            key: 'machine_uuid',
            is_uuid: true,
            is_list: false
        }
    ]
};

/*
 {
     type: 'node.primitive.machine-primitive.',
     parameters: {
         machine_uuid: 'some-uuid'
     }
 }
*/

const delay_template = {
    type: 'node.primitive.delay.',
    name: 'Delay', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        {
            type: '<number>',
            key: 'duration',
            is_uuid: false,
            is_list: false
        }
    ]
};

const gripper_template = {
    type: 'node.primitive.gripper.',
    name: 'Gripper', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        {
            type: '<number>',
            key: 'position',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'speed',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'effort',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<enum>',
            key: 'semantic',
            is_uuid: false,
            is_list: false,
            enum_values: [
                'ambiguous',
                'grasping',
                'releasing'
            ]
        },
        {
            type: 'node.pose.thing.',
            key: 'thing_uuid',
            is_uuid: true,
            is_list: false
        }
    ]
};

const machine_initialize_template = {
    type: 'node.primitive.machine-primitive.machine-initialize.',
    name: 'Machine Initialize', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        {
            type: 'node.machine.',
            key: 'machine_uuid',
            is_uuid: true,
            is_list: false
        }
    ]
};

const machine_start_template = {
    type: 'node.primitive.machine-primitive.machine-start.',
    name: 'Machine Start', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        {
            type: 'node.machine.',
            key: 'machine_uuid',
            is_uuid: true,
            is_list: false
        }
    ]
};

const machine_stop_template = {
    type: 'node.primitive.machine-primitive.machine-stop.',
    name: 'Machine Stop', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        {
            type: 'node.machine.',
            key: 'machine_uuid',
            is_uuid: true,
            is_list: false
        }
    ]
};

const machine_wait_template = {
    type: 'node.primitive.machine-primitive.machine-wait.',
    name: 'Machine Wait', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        {
            type: 'node.machine.',
            key: 'machine_uuid',
            is_uuid: true,
            is_list: false
        }
    ]
};

const move_trajectory_template = {
    type: 'node.primitive.move-trajectory.',
    name: 'Move Trajectory', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        {
            type: '<boolean>',
            key: 'manual_safety',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.trajectory.',
            key: 'trajectory_uuid',
            is_uuid: true,
            is_list: false
        }
    ]
};

const move_unplanned_template = {
    type: 'node.primitive.move-unplanned.',
    name: 'Move Unplanned', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        {
            type: '<boolean>',
            key: 'manual_safety',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<number>',
            key: 'velocity',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<enum>',
            key: 'move_type',
            is_uuid: false,
            is_list: false,
            enum_values: [
                'joint',
                'ee_ik'
            ]
        },
        {
            type: 'node.pose.waypoint.location.',
            key: 'location_uuid',
            is_uuid: true,
            is_list: false
        }
    ]
};

/***************************************************************** 
* Hierarchical
*****************************************************************/

const hierarchical_template = {
    type: 'node.primitive.hierarchical.',
    name: 'Hierarchical', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<all-primitives>',
            key: 'primitives',
            is_uuid: false,
            is_list: true
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        // empty
    ]
};

/***************************************************************** 
* Control Flow
*****************************************************************/

const breakpoint_template = {
    type: 'node.primitive.breakpoint',
    name: 'Breakpoint', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        //empty
    ]
};

const skillCall_template = {
    type: 'node.primitive.skill-call',
    name: 'Skill Call', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        {
            type: 'node.primitive.skill.',
            key: 'skill_uuid',
            is_uuid: true,
            is_list: false
        }
        /* Plus it shadow params skill args (key = arg.name, value = actual value) */
    ]
};

const loop_template = {
    type: 'node.primitive.hierarchical.loop.',
    name: 'Loop', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<all-primitives>',
            key: 'primitives',
            is_uuid: false,
            is_list: true
        },
        {
            type: '<all-conditions>', //not currently used, set to null
            key: 'condition',
            is_uuid: false,
            is_list: false
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        // empty
    ]
};

/***************************************************************** 
* Skills
*****************************************************************/

const skill_template = {
    type: 'node.primitive.hierarchical.skill.',
    name: 'Skill', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'nodes.skill-argument.',
            key: 'arguments',
            is_uuid: false,
            is_list: true
        },
        {
            type: '<all-primitives>',
            key: 'primitives',
            is_uuid: false,
            is_list: true
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        //empty
    ]
};

const machine_blocking_process_template = {
    type: 'node.primitive.hierarical.skill.machine-blocking-process.',
    name: 'Machine Blocking Process Skill', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'nodes.skill-argument.',
            key: 'arguments',
            is_uuid: false,
            is_list: true
        },
        {
            type: '<all-primitives>',
            key: 'primitives',
            is_uuid: false,
            is_list: true
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        //empty
    ]
};

const simple_pick_and_place_template = {
    type: 'node.primitive.hierarchical.skill.simple-pick-and-place',
    name: 'Simple Pick and Place Skill', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'nodes.skill-argument.',
            key: 'arguments',
            is_uuid: false,
            is_list: true
        },
        {
            type: '<all-primitives>',
            key: 'primitives',
            is_uuid: false,
            is_list: true
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        //empty
    ]
};

const close_gripper_template = {
    type: 'node.primitive.hierarchical.skill.close-gripper.',
    name: 'Close Gripper Skill', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<all-primitives>',
            key: 'primitives',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'nodes.skill-argument.', // hint this one doesn't actually take args
            key: 'arguments',
            is_uuid: false,
            is_list: true
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        // empty
    ]
};

const open_gripper_template = {
    type: 'node.primitive.hierarchical.skill.open-gripper.',
    name: 'Open Gripper Skill', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<all-primitives>',
            key: 'primitives',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'nodes.skill-argument.', //hint this one doesn't actually take args
            key: 'arguments',
            is_uuid: false,
            is_list: true
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        // empty
    ]
};

const initialize_template = {
    type: 'node.primitive.hierarchical.skill.initialize.',
    name: 'Initialize Skill', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<all-primitives>',
            key: 'primitives',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'nodes.skill-argument.',
            key: 'arguments',
            is_uuid: false,
            is_list: true
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        // empty
    ]
};

/***************************************************************** 
* Program
*****************************************************************/

const program_template = {
    type: 'node.primitive.hierarchical.program',
    name: 'Program', // generated display name
    meta_data: [ // defining instance meta data
        {
            type: '<string>',
            key: 'type'
        },
        {
            type: '<string>',
            key: 'uuid'
        },
        {
            type: '<string>',
            key: 'name'
        },
        {
            type: '<boolean>',
            key: 'editable'
        },
        {
            type: '<boolean>',
            key: 'deleteable'
        },
        {
            type: '<string>',
            key: 'description'
        }
    ],
    fields: [ // defining instance fields
        {
            type: '<parameters>',
            key: 'parameters',
            is_uuid: false,
            is_list: false
        },
        {
            type: '<all-primitives>',
            key: 'primitives',
            is_uuid: false,
            is_list: true
        },
        {
            type: 'node.context.environment.',
            key: 'environment',
            is_uuid: false,
            is_list: false
        },
        {
            type: 'node.primitive.hierarchical.skill.',
            key: 'skills',
            is_uuid: false,
            is_list: true
        }
    ],
    parameters: [ // define primitive parameters that can be filled in by the user
        // empty
    ]
};

//=================================================================

/// Export
const fields = {
    node_template,
    thingType_template,
    gradeType_template,
    skillArgument_template,
    placholder_template,
    pose_template,
    position_template,
    orientation_template,
    joints_template,
    waypoint_template,
    location_template,
    machine_template,
    thing_template,
    region_template,
    cube_region_template,
    sphere_region_template,
    trajectory_template,
    trace_template,
    environmentNode_template,
    reachSphere_template,
    collisionMesh_template,
    occupancyZone_template,
    pinchPoint_template,
    context_template,
    environment_template,

    primitive_template,
    machine_primitive_template,
    delay_template,
    gripper_template,
    machine_initialize_template,
    machine_start_template,
    machine_stop_template,
    machine_wait_template,
    move_trajectory_template,
    move_unplanned_template,
    hierarchical_template,
    breakpoint_template,
    skillCall_template,
    loop_template,

    skill_template,
    machine_blocking_process_template,
    simple_pick_and_place_template,
    program_template,
    close_gripper_template,
    open_gripper_template,
    initialize_template
};

export default fields;