/*
 * Fake data attempts to expess EvDscript's datastructure without
 * need for ROS connection. Useful for development and expectation
 * documentation.
 */

const NUM_THING_TYPES = 5;
const NUM_GRADE_TYPES = 3;
const NUM_WAYPOINTS = 20;
const NUM_LOCATIONS = 20;
const NUM_THINGS = 10;
const NUM_TRAJECTORIES = 4;
const NUM_COLLISION_MESHES = 2;
const NUM_PINCH_POINTS = 5;

/***************************************************************** 
* Type Declaration
* - ThingType
*****************************************************************/

let thingTypes = [];
for (let i=0; i<NUM_THING_TYPES; i++) {
    thingTypes.push({
        type: 'node.thing-type.',
        uuid: `thing-type-js-${i}`,
        name: `Thing-Type-${i}`,
        deleteable: false,
        editable: true,
        description: 'Some descriptor string',

        mesh_id: 'package://app/meshes/3DBenchy.stl', // This specific pathing is subject to change (using robot-scene variant for now)
        is_safe: true,  // Checklist that creates this aggregate value
        weight: 1.0 // kg in one g (evd will not work natively on the moon)
        // Assume scale is (1,1,1) - meshes sized correctly
    });
}

let gradeTypes = [];
for (let i=0; i<NUM_GRADE_TYPES; i++) {
    gradeTypes.push({
        uuid: `grade-type-js-${i}`,
        name: `Grade-Type-${i}`,
        deleteable: false,
        editable: true,
        description: 'Some descriptor string'
    });
}

/***************************************************************** 
*  Positional Data
* - Waypoint
* - Location
* - Thing (instances)
*****************************************************************/

let waypoints = [];
for (let i=0; i<NUM_WAYPOINTS; i++) {
    waypoints.push({
        uuid: `waypoint-js-${i}`,
        name: `Waypoint-${i}`,
        deleteable: true,
        editable: true,
        description: 'Some descriptor string (optional)', // could be ''

        joints: {
            uuid: `joints-js-waypoint-${i}`,
            name: '',
            deleteable: false,
            editable: true,
            description: 'Some descriptor string (optional)', // could be ''

            joint_positions: [0,0,0], // or null
            joint_names: ['j1','j2','j3'], // or null
            reachable: false, // better than having to check the array to generate the flag
            length: 3 // this is enforced on the backend for positions and names
        },
        position: {
            uuid: `position-js-waypoint-${i}`,
            name: '',
            deleteable: false,
            editable: true,
            description: 'Some descriptor string (optional)', // could be ''

            x: 0,
            y: 0,
            z: 0
        },
        orientation: {
            uuid: `orientation-js-waypoint-${i}`,
            name: '',
            deleteable: false,
            editable: true,
            description: 'Some descriptor string (optional)', // could be ''

            x: 0,
            y: 0,
            z: 0,
            w: 1
        }
    });
}

let locations = [];
for (let i=0; i<NUM_LOCATIONS; i++) {
    locations.push({
        uuid: `location-js-${i}`,
        name: `Location-${i}`,
        deleteable: ! (i === 3),
        editable: ! (i % 4 === 0),
        description: 'Some descriptor string (optional)', // could be ''

        joints: {
            uuid: `joints-js-location-${i}`,
            name: '',
            deleteable: false,
            editable: true,
            description: 'Some descriptor string (optional)', // could be ''

            joint_positions: [0,0,0], // or null
            joint_names: ['j1','j2','j3'], // or null
            reachable: false, // better than having to check the array to generate the flag
            length: 3 // this is enforced on the backend for positions and names
        },
        position: {
            uuid: `position-js-location-${i}`,
            name: '',
            deleteable: false,
            editable: true,
            description: 'Some descriptor string (optional)', // could be ''

            x: 0,
            y: 0,
            z: 0
        },
        orientation: {
            uuid: `orientation-js-location-${i}`,
            name: '',
            deleteable: false,
            editable: true,
            description: 'Some descriptor string (optional)', // could be ''

            x: 0,
            y: 0,
            z: 0,
            w: 1
        }
    });
}

let things = [];
for (let i=0; i<NUM_THINGS; i++) {
    things.push({
        uuid: `thing-js-${i}`,
        name: `Thing-${i}`,
        deleteable: true,
        editable: true,
        description: 'Some descriptor string (optional)', // could be ''

        thing_type_uuid: thingTypes[0].uuid, // gets a thing_type from other fake data
        position: {
            uuid: `position-js-thing-${i}`,
            name: '',
            deleteable: false,
            editable: true,
            description: 'Some descriptor string (optional)', // could be ''

            x: 0,
            y: 0,
            z: 0
        },
        orientation: {
            uuid: `orientation-js-thing-${i}`,
            name: '',
            deleteable: false,
            editable: true,
            description: 'Some descriptor string (optional)', // could be ''

            x: 0,
            y: 0,
            z: 0,
            w: 1
        }
    });
}

/***************************************************************** 
* Regions
*****************************************************************/

// Probably don't visualize all of fields of regions
const generic_region = {
    uuid: `region-js-0`,
    name: `Region-0`,
    deleteable: true,
    editable: true,
    description: 'Some descriptor string (optional)', // could be ''

    // position is known in a default region, just uncertainty in orientation
    center_position: {
        uuid: `position-js-region-0`,
        name: '',
        deleteable: false,
        editable: true,
        description: 'Some descriptor string (optional)', // could be ''

        x: 0,
        y: 0,
        z: 0
    },
    center_orientation: {
        uuid: `orientation-js-region-0`,
        name: '',
        deleteable: false,
        editable: true,
        description: 'Some descriptor string (optional)', // could be ''

        x: 0,
        y: 0,
        z: 0,
        w: 1
    },
    free_orientation: false,
    uncertainty_orientation_limit: 1.0,
    // if free_orientation then it is null (don't care case)
    // if not free_orientation it can be null meaning use the center_orientation as target
        // target relative to other node for qaternion distance
    // otherwise switch over to alt target as the relative node for distance calc
    uncertainty_orientation_alt_target: {
        uuid: `orientation-js-region-alt-0`,
        name: '',
        deleteable: false,
        editable: true,
        description: 'Some descriptor string (optional)', // could be ''

        x: 0,
        y: 0,
        z: 0,
        w: 1
    }
};

// Set to free orientation for simplicity only - can have orientation constraints
const cube_region = {
    uuid: `cube-region-js-1`,
    name: `Cube-Region-1`,
    deleteable: false,
    editable: false,
    description: 'Some descriptor string (optional)', // could be ''

    center_position: {
        uuid: `position-js-cube-region-1`,
        name: '',
        deleteable: false,
        editable: true,
        description: 'Some descriptor string (optional)', // could be ''

        x: 0,
        y: 0,
        z: 0
    },
    center_orientation: {
        uuid: `orientation-js-cube-region-1`,
        name: '',
        deleteable: false,
        editable: true,
        description: 'Some descriptor string (optional)', // could be ''

        x: 0,
        y: 0,
        z: 0,
        w: 1
    },
    free_orientation: true,
    uncertainty_orientation_limit: null,
    uncertainty_orientation_alt_target: null,
    uncertainty_x: 1.0,
    uncertainty_y: 1.0,
    uncertainty_z: 1.0
};

// Set to free orientation for simplicity only - can have orientation constraints
const sphere_region = {
    type: 'node.pose.region.sphere-region.',
    uuid: `sphere-region-js-2`,
    name: `Sphere-Region-2`,
    deleteable: true,
    editable: false,
    description: 'Some descriptor string (optional)', // could be ''

    center_position: {
        type: 'node.position.',
        uuid: `position-js-sphere-region-3`,
        name: '',
        deleteable: false,
        editable: true,
        description: 'Some descriptor string (optional)', // could be ''

        x: 0,
        y: 0,
        z: 0
    },
    center_orientation: {
        type: 'node.orientation.',
        uuid: `orientation-js-sphere-region-3`,
        name: '',
        deleteable: false,
        editable: true,
        description: 'Some descriptor string (optional)', // could be ''

        x: 0,
        y: 0,
        z: 0,
        w: 1
    },
    free_orientation: true,
    uncertainty_orientation_limit: null,
    uncertainty_orientation_alt_target: null,
    uncertainty_radius: 1.0
};

let regions = [
    generic_region,
    cube_region,
    sphere_region
];

/***************************************************************** 
* Machines
* - Machine
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

const machine_generator = {
    type: 'node.machine.',
    uuid: `machine-js-generator`,
    name: `Machine-Generator`,
    deleteable: false,
    editable: false,
    description: 'Some descriptor string (optional)', // could be ''

    process_time: 5, //sec
    inputs: {}, // make this a tuple as value of keys
    outputs: {
        'thing-type-js-0': [
            {
                region_uuid: cube_region.uuid,
                quantity: 1
            }
        ]
    },
    mesh_id: 'package:/app/meshes/3d_printer.fbx',
    pose_offset: { // Local transform
        type: 'node.pose.',
        uuid: `pose-js-machine-0`,
        name: `Pose-0`,
        deleteable: false,
        editable: false,
        description: 'Some descriptor string (optional)', // could be ''

        position: {
            type: 'node.position.',
            uuid: `position-js-machine-0`,
            name: `Position-0`,
            deleteable: false,
            editable: false,
            description: 'Some descriptor string (optional)', // could be ''

            x:0,
            y:0,
            z:0
        },
        orientation: {
            type: 'node.orientation.',
            uuid: `orientation-js-machine-0`,
            name: `Orientation-0`,
            deleteable: false,
            editable: false,
            description: 'Some descriptor string (optional)', // could be ''

            x:0,
            y:0,
            z:0,
            w:1
        }
    },
    link: 'app',
    collision_mesh_uuid: 'collision-mesh-js-0'
}

const machine_consumer = {
    type: 'node.machine.',
    uuid: `machine-js-consumer`,
    name: `Machine-Consumer`,
    deleteable: false,
    editable: false,
    description: 'Some descriptor string (optional)', // could be ''

    process_time: 2, //sec
    inputs: {
        'thing-type-js-1': [
            {
                region_uuid: generic_region.uuid,
                quantity: 1
            }
        ]
    }, 
    outputs: { },
    mesh_id: 'package:/app/meshes/3d_printer.fbx',
    pose_offset: { // Local transform
        type: 'node.pose.',
        uuid: `pose-js-machine-1`,
        name: `Pose-1`,
        deleteable: false,
        editable: false,
        description: 'Some descriptor string (optional)', // could be ''

        position: {
            type: 'node.position.',
            uuid: `position-js-machine-1`,
            name: `Position-1`,
            deleteable: false,
            editable: false,
            description: 'Some descriptor string (optional)', // could be ''

            x:0,
            y:0,
            z:0
        },
        orientation: {
            type: 'node.orientation.',
            uuid: `orientation-js-machine-1`,
            name: `Orientation-1`,
            deleteable: false,
            editable: false,
            description: 'Some descriptor string (optional)', // could be ''

            x:0,
            y:0,
            z:0,
            w:1
        }
    },
    link: 'app',
    collision_mesh_uuid: 'collision-mesh-js-1'
};

const machine_transformer = {
    type: 'node.machine.',
    uuid: `machine-js-transformer`,
    name: `Machine-Transformer`,
    deleteable: false,
    editable: false,
    description: 'Some descriptor string (optional)', // could be ''

    process_time: 2, //sec
    inputs: {
        'thing-type-js-0': [
            {
                region_uuid: sphere_region.uuid,
                quantity: 1
            }
        ]
    }, 
    outputs: {
        'thing-type-js-1': [
            {
                region_uuid: sphere_region.uuid,
                quantity: 1
            }
        ]
    },
    mesh_id: 'package:/app/meshes/3d_printer.fbx',
    pose_offset: { // Local transform
        type: 'node.pose.',
        uuid: `pose-js-machine-2`,
        name: `Pose-2`,
        deleteable: false,
        editable: false,
        description: 'Some descriptor string (optional)', // could be ''

        position: {
            type: 'node.position.',
            uuid: `position-js-machine-2`,
            name: `Position-2`,
            deleteable: false,
            editable: false,
            description: 'Some descriptor string (optional)', // could be ''

            x:0,
            y:0,
            z:0
        },
        orientation: {
            type: 'node.orientation.',
            uuid: `orientation-js-machine-2`,
            name: `Orientation-2`,
            deleteable: false,
            editable: false,
            description: 'Some descriptor string (optional)', // could be ''

            x:0,
            y:0,
            z:0,
            w:1
        }
    },
    link: 'app',
    collision_mesh_uuid: 'collision-mesh-js-2' // could be null
};

let machines = [
    machine_generator,
    machine_consumer,
    machine_transformer
];

/***************************************************************** 
* Trajectories
* - Trajectory
* - Trace
* - TraceDataPoint
* - Grade
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

let trajectories = [];
for (let i=0; i<NUM_TRAJECTORIES; i++) {
    trajectories.push({
        type: 'node.trajectory.',
        uuid: `trajectory-js-${i}`,
        name: `Trajectory-${i}`,
        deleteable: false,
        editable: false,
        description: 'Some descriptor string (optional)', // could be ''

        start_location_uuid: locations[0].uuid,
        end_location_uuid: locations[1].uuid,
        waypoint_uuids: [
            // presumably filled with arbitrary (ordered) list of waypoint uuids
        ],
        trace: { // trace can be null, supplied by backend when planning is complete
            type: 'node.trace.',
            uuid: `trace-js-trajectory-${i}`,
            name: `Trace-${i}`,
            deleteable: false,
            editable: false,
            description: 'Some descriptor string (optional)', // could be ''

            time_data: [0, /*...*/], // each timestep, time from relative start
            joint_names: ['j1','j2','j3','j4','j5','j6'], // name for each joint corresponding to inner array on joint_data
            joint_data: [[0,0,0,0,0,0]], // joint state of robot at each timestep (NOTE that this is not an EvD Joints just a simple list)
            tf_data: {
                'ee_link': [ // Pose at that frame at each timestep (NOTE that this is not an EvD Pose just a simple dict)
                    {
                        position: {x:0, y:0, z:0},
                        orientation: {x:0, y:0, z:0, w:0},
                    }
                    /*...*/
                ],
                // ... (e.g keys for joint_tf_frame_1, gripper_tf_frame_1)
            },
            // Keys are predefined grade type uuids
            grades: {
                'grade_max_velocity_uuid': [0.5, /*...*/] // list of grade values from 0 to 1 (for all grades), though semantics of that is dependent on the grader
                //... (e.g. grade_min_velocity, collision_proximity)
            },

            // length of time_data, joint_data, each value list in tf_data and in graders should be equal

            duration: 1, // sec (full time to complete action Note approximate)
            end_effector_path: 'ee_link', //the frame of the end-effector,
            joints_paths: ['joint_tf_frame_1','joint_tf_frame_2'], // tf-frames of arm joints being tracked
            tool_paths: ['gripper_tf_frame_1'], // tf-frames for grippers
            component_paths: ['thing_tf_frame'], // (optional / may no longer be needed)
        },
        velocity: 0.5, // suggestion (user configured)
        move_type: 'joint' // or 'ee_ik'
    });
}

/***************************************************************** 
* Environment Objects
* - Reach Sphere
* - Collision Mesh
* - Occupancy Zone
* - Pinch Point
*****************************************************************/

let reachSphere = {
    type: 'node.environment-node.reach-sphere.',
    uuid: `reach-sphere-js-0`,
    name: `ReachSphere`,
    deleteable: false,
    editable: false,
    description: 'Some descriptor string (optional)', // could be ''

    radius: 1.0,
    offset: {
        type: 'node.position.',
        uuid: `position-js-reach-sphere-offset-0`,
        name: '',
        deleteable: false,
        editable: true,
        description: 'Some descriptor string (optional)', // could be ''

        x: 0,
        y: 0,
        z: 0
    }
};

let collisionMeshes = [];
for (let i=0; i<NUM_COLLISION_MESHES; i++) {
    collisionMeshes.push({
        type: 'node.environment-node.collision-mesh.',
        uuid: `collision-mesh-js-${i}`,
        name: `CollisionMesh-${i}`,
        deleteable: false,
        editable: false,
        description: 'Some descriptor string (optional)', // could be ''

        mesh_id: 'default.stl',
        pose_offset: { // Local transform
            type: 'node.pose.',
            uuid: `pose-js-collision-mesh-${i}`,
            name: `Pose-${i}`,
            deleteable: false,
            editable: false,
            description: 'Some descriptor string (optional)', // could be ''

            position: {
                type: 'node.position.',
                uuid: `position-js-collision-mesh-${i}`,
                name: `Position-${i}`,
                deleteable: false,
                editable: false,
                description: 'Some descriptor string (optional)', // could be ''

                x:0,
                y:0,
                z:0
            },
            orientation: {
                type: 'node.orientation.',
                uuid: `orientation-js-collision-mesh-${i}`,
                name: `Orientation-${i}`,
                deleteable: false,
                editable: false,
                description: 'Some descriptor string (optional)', // could be ''

                x:0,
                y:0,
                z:0,
                w:1
            }
        },
        link: 'some_frame' // Can be used in the TF tree as a frame (could be '' or 'app' which means 'world')
    });
}

let occupancyZones = [
    {
        type: 'node.environment-node.occupancy-zone.',
        uuid: `occupancy-zone-js-0`,
        name: `Human Occupancy Zone`,
        deleteable: false,
        editable: false,
        description: 'Some descriptor string (optional)', // could be ''

        occupancy_type: 'human', // human is conceptualized as a box
        position_x: 0, // 'center'
        position_z: 0, // 'center'
        scale_x: 1,    // 'radius' (e.g. total width is 2x this)
        scale_y: 0.5,  // 'radius' (e.g. total width is 2x this)
        height: 0 // typically a negative value (wherever ground is)
    },
    {
        type: 'node.environment-node.occupancy-zone.',
        uuid: `occupancy-zone-js-1`,
        name: `Human Occupancy Zone`,
        deleteable: false,
        editable: false,
        description: 'Some descriptor string (optional)', // could be ''

        occupancy_type: 'robot', // robot is ellipse "cast as shadow of reach-sphere"
        position_x: 0, // 'center'
        position_z: 0, // 'center'
        scale_x: 0.5,  // 'radius' (e.g. total width is 2x this)
        scale_y: 0.5,  // 'radius' (e.g. total width is 2x this)
        height: 0 // typically a negative value (wherever ground is)
    }
];

let pinchPoints = [];
for (let i=0; i<NUM_PINCH_POINTS; i++) {
    pinchPoints.push({
        type: 'node.environment-node.pinch-point.',
        uuid: `pinch-points-js-${i}`,
        name: `Pinch Point ${i}`,
        deleteable: false,
        editable: false,
        description: 'Some descriptor string (optional)', // could be ''

        // conceptualized as a cylinder
        axis: 'x', // 'x', 'y', or 'z' ~ aligned along one of these axes
        offset: { // positional offset from link
            type: 'node.position.',
            uuid: `position-js-pinch-point-${i}`,
            name: `Position-${i}`,
            deleteable: false,
            editable: false,
            description: 'Some descriptor string (optional)', // could be ''

            x:0,
            y:0,
            z:0
        },
        link: 'some_robot_frame',
        radius: 0.25,
        length: 0.5
    });
}


/***************************************************************** 
* Robot Control Server
* - Token Blob
* - RCS interface
* - ROS Msgs
*****************************************************************/

/*
 * Subscribe to 'robot_control_server/at_start' -> Bool     (True when program starts)
 * Subscribe to 'robot_control_server/at_end' -> Bool       (True when program ends)
 * Subscribe to 'robot_control_server/lockout' -> Bool      (True while program running)
 * Subscribe to 'robot_control_server/status' -> ProgramRunnerStatus
 * Subscribe to 'robot_control_server/tokens' -> String (JSON blob)
 * Subscribe to 'robot_control_server/error' -> String      (A human readable error message)
 * 
 * Publish to 'robot_control_server/play' <- Empty "{}"     (Commands server to run)
 * Publish to 'robot_control_server/stop' <- Empty          
 * Publish to 'robot_control_server/pause' <- Empty         (This one might not work in final version?)
 * Publish to 'robot_control_server/reset' <- Empty
 * 
 * Service 'robot_control_server/set_root_node' <- SetRootNodeRequest, SetRootNodeResponse
 * Service 'robot_control_server/get_root_node' <- GetRootNodeRequest, GetRootNodeResponse
 */

let rcs_tokens = {
    robot: {
        type: 'robot',
        state: {
            position: {x:0, y:0, z:0}, // EE relative to world, these values can also be '?' at start
            orientation: {x:0, y:0, z:0, w:0},
            joints: ['?'], // current joint state of robot or ['?'] if unknown
            gripper: {
                position: 0, //current gripper state or '?' if unknown
                grasped_thing: null, // either null if no thing grasped or string uuid of thing being grasped
                ambigous_flag: false // true if gripper is ever actuated without proper semantics and thing association (except in initialize)
            }
        }
    },
    'machine-js-0': {
        type: 'machine',
        state: '?'  // '?' at start, otherwise of [idle, pause, running, error]
    },
    /*...*/ //for all machines
    'thing-js-0': {
        type: 'thing',
        state: {
            position: {x:0, y:0, z:0}, // or ? for each
            orientation: {x:0, y:0, z:0}
        }
    },
    /*...*/ //for all things (including the ones going to be generated by machines)
};

const ProgramRunnerStatus = {
    uuid: '', //string of active primitive
    //TODO: define status/state structure for highlighting from this UUID
    start_time: 0, //floating point start time from time.time() (-1 if not set)
    previous_time: 0.1, // last timestep time (-1 if not set)
    current_time: 0.3,  // current timestep time (-1 if not set)
    stop_time: -1       // final execution time (only set at the end)
};

const SetRootNodeRequest = {
    uuid: '', //string uuid of node in program to be considered root for execution if empty string then defaults to program
};

const SetRootNodeResponse = {
    status: true, // boolean indicating operation was succesful
    message: '', //human readable error message if node was not found or otherwise unable to be set
};

const GetRootNodeRequest = {
    // Purposefully empty
};

const GetRootNodeResponse = {
    uuid: '' //string uuid of node in program to be considered root for execution if empty string then defaults to program
};

/***************************************************************** 
* Issue Server
* - Issues
* - Pending Jobs
*****************************************************************/

const GetIssuesRequest = {
    filter_by_source: false, //tells server to look at source and id fields, just set to false for frontend
    source: '', //some string like "data_server", probably not useful for frontend display
    ids: [], //of strings, again not useful for frontend

    filter_by_level: true, //tells filter to get only a degree of error (though error registration is up to the job register)
    level: 'error' //arbitrary level scheme (dependent on what the registered issues are)
};

const GetIssuesResponse = {
    issues: [
        { // Issues.msg
            source: 'string',
            id: 'string',
            level: 'error', // can be LEVEL_NOTE='note', LEVEL_WARN='warn', LEVEL_ERROR='error'
            data: 'string -> JSON blob', // the json blob is up to the issuer so not sure what this will look like yet
            human_message: 'string' // some human description of the issue at hand
        }
    ]
};

const GetPendingJobsRequest = {}; // purposefully empty

const GetPendingJobrResponse = {
    sources: [], //of strings
    ids: [
        [] // of strings where outer list is sources and inner list is ids
    ],
    human_messages: [
        [] // of strings where outer list is sources and inner list is human readable messages of jobs
    ],
    data: [
        [] // of strings where outer list is sources and inner list is a JSON blob that the job issuer defines
    ],
    status: true, // issue server response status probably will never be false
    message: '' // or error message if status is false
};

/***************************************************************** 
* Program
* - Program (object)
* - Skills
*   - (Predefined)
*   - (User-defined)
* - Primitives
*   - Various Predefined
* - Control Flow
*****************************************************************/

let program = {
    type: 'node.primitive.skill.program.',
    uuid: `program-js`,
    name: ``,
    deleteable: false,
    editable: true,
    description: 'Some descriptor string (optional)', // could be ''

    
    // Andy's musings
    

    macro_calls: [
        {
            uuid: 'macro_call_1',
            name: 'My Macro Call',
            deleteable: false,
            editable: true,
            description: 'Some descriptor string (optional)', // could be ''
            macro_uuid: 'macro_1_js',
            macro_args: [ // probably this should be key-value + have typing info
                'trajectory_1_uuid',
                'trajectory_2_uuid'
            ]
        }
    ],

    macro_library: [
        {
            uuid: 'macro_1_js',
            name: 'My Macro',
            deleteable: false,
            editable: true,
            description: 'Some descriptor string (optional)', // could be ''
            fields: [
                {
                    type:'node.trajectory.',
                    key:'trajectory_uuid', // This is the key that it inserts into the primitives
                    value:'real_trajectory_uuid' // This is the name that gets swapped in
                },
                {
                    type:'node.trajectory.',
                    key:'trajectory_uuid', // This is the key that it inserts into the primitives
                    value:'second_trajectory_uuid' // This is the name that gets swapped in
                }
            ],
            
            primitives: [
                {
                    type: 'node.primitive.move-trajectory.',
                    uuid: `move-trajectory-primitive-js`,
                    name: ``,
                    deleteable: false,
                    editable: true,
                    description: 'Some descriptor string (optional)', // could be ''

                    trajectory_uuid: 'real_trajectory_uuid'
                    //TODO
                },
                {
                    type: 'node.primitive.move-trajectory.',
                    uuid: `move-trajectory-primitive-js`,
                    name: ``,
                    deleteable: false,
                    editable: true,
                    description: 'Some descriptor string (optional)', // could be ''

                    trajectory_uuid: 'second_trajectory_uuid'
                    //TODO
                },
            ]
        }
    ],

    //TODO add an ownership
    
    environment: {
        reach_sphere: reachSphere,
        pinch_points: pinchPoints,
        collision_meshes: collisionMeshes,
        occupancy_zones: occupancyZones,
        locations: locations,
        machines: machines,
        things: things,
        waypoints: waypoints,
        trajectories: trajectories,
        thing_types: thingTypes,
        regions: regions,
        grade_types: gradeTypes
    },
    
    primitives: [
        // Assume we want a simple non-looping pick-and-place task

        {
            uuid: `initialize-skill-js`,
            name: ``,
            deleteable: false,
            editable: true,
            description: 'Some descriptor string (optional)', // could be ''

            primitives: [
                {
                    uuid: `machine-initialize-machine-primitive-js`,
                    name: ``,
                    deleteable: false,
                    editable: true,
                    description: 'Some descriptor string (optional)', // could be ''

                    machine_uuid: 'machine-js-0' //some machine uuid
                },
                /*...*/ //for all machines
                {
                    uuid: `move-unplanned-primitive-js`,
                    name: ``,
                    deleteable: false,
                    editable: true,
                    description: 'Some descriptor string (optional)', // could be ''

                    //TODO
                },
                {
                    uuid: `open-gripper-skill-js`,
                    name: ``,
                    deleteable: false,
                    editable: true,
                    description: 'Some descriptor string (optional)', // could be ''

                    primitives: [
                        {
                            uuid: `gripper-primitive-js`,
                            name: ``,
                            deleteable: false,
                            editable: true,
                            description: 'Some descriptor string (optional)', // could be ''

                            // TODO
                        }
                    ]
                },
            ]
        },
        {
            uuid: `simple_pick_and_place-skill-js`,
            name: ``,
            deleteable: false,
            editable: true,
            description: 'Some descriptor string (optional)', // could be ''

            primitives: [
                {
                    type: 'node.primitive.move-trajectory.',
                    uuid: `move-trajectory-primitive-js`,
                    name: ``,
                    deleteable: false,
                    editable: true,
                    description: 'Some descriptor string (optional)', // could be ''

                    trajectory_uuid: 'trajectory-js-0'
                    //TODO
                },
                {
                    uuid: `close-gripper-skill-js`,
                    name: ``,
                    deleteable: false,
                    editable: true,
                    description: 'Some descriptor string (optional)', // could be ''

                    primitives: [
                        {
                            uuid: `gripper-primitive-js`,
                            name: ``,
                            deleteable: false,
                            editable: true,
                            description: 'Some descriptor string (optional)', // could be ''

                            // TODO
                        }
                    ]
                },
                {
                    uuid: `move-trajectory-primitive-js`,
                    name: ``,
                    deleteable: false,
                    editable: true,
                    description: 'Some descriptor string (optional)', // could be ''

                    // TODO
                },
                {
                    uuid: `open-gripper-skill-js`,
                    name: ``,
                    deleteable: false,
                    editable: true,
                    description: 'Some descriptor string (optional)', // could be ''

                    primitives: [
                        {
                            uuid: `gripper-primitive-js`,
                            name: ``,
                            deleteable: false,
                            editable: true,
                            description: 'Some descriptor string (optional)', // could be ''

                            // TODO
                        }
                    ]
                },
            ]
        },
        {
            uuid: `move-trajectory-primitive-js`,
            name: `Retract to Home`,
            deleteable: false,
            editable: true,
            description: 'Some descriptor string (optional)', // could be ''

            // TODO
        }
    ]
};



/***************************************************************** 
* Data Server - Comm
*****************************************************************/
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

// TODO actually write the full list
// also if you have any suggestions on additional features
// I also templated out all data objects but don't currently have an endpoint 
// on the data server to support this
let primitive_library = [
    {
        type: 'node.primitive.move-trajectory',
        name: 'Move Trajectory',
        meta_data: [
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
                key: 'deleteable'
            },
            {
                type: '<boolean>',
                key: 'editable'
            },
            {
                type: '<string>',
                key: 'description'
            }
        ],
        fields: [
            {
                type:'node.trajectory.',
                name:'trajectory_uuid',
                is_uuid: true,
                is_list: false
            }
        ]
    },
    {
        type: 'node.primitive.gripper',
        name: 'Gripper',
        meta_data: [/*...*/], //same as before
        fields: [
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
                    'ambigous',
                    'grasping',
                    'releasing'
                ]
            },
            {
                type: 'node.thing.',
                key: 'thing_uuid',
                is_uuid: true,
                is_list: false
            }
        ]
    }
]


//=================================================================

/// Export
const fields = {

    // all of these in environment
    waypoints,
    locations,
    machines,
    thingTypes,
    gradeTypes,
    things,
    regions,
    trajectories,
    reachSphere,
    collisionMeshes,
    occupancyZones,
    pinchPoints,

    // evd_program
    program,

    // Robot Control Server
    rcs_tokens,
    ProgramRunnerStatus,
    SetRootNodeRequest,
    SetRootNodeResponse,
    GetRootNodeRequest,
    GetRootNodeResponse,

    // Issue Server
    GetIssuesRequest,
    GetIssuesResponse,
    GetPendingJobsRequest,
    GetPendingJobrResponse
};

export default fields;