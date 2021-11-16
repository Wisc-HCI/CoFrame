/*
 * Provides an arbitrary set of data. Linking between objects is not enforced
 */

const NUM_THING_TYPES = 5;
const NUM_GRADE_TYPES = 3;
const NUM_WAYPOINTS = 20;
const NUM_LOCATIONS = 20;
const NUM_THINGS = 10;
const NUM_TRAJECTORIES = 4;
const NUM_COLLISION_MESHES = 2;
const NUM_PINCH_POINTS = 5;
const NUM_PLACEHOLDERS = 3;

/*****************************************************************
* Type Declaration
* - ThingType
* - GradeType
*****************************************************************/

let thingTypes = [];
for (let i = 0; i < NUM_THING_TYPES; i++) {
    thingTypes.push({
        type: 'node.thing-type.',
        uuid: `thing-type-js-${i}`,
        name: `ThingType ${i} Name`,
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
for (let i = 0; i < NUM_GRADE_TYPES; i++) {
    gradeTypes.push({
        type: 'node.grade-type.',
        uuid: `grade-type-js-${i}`,
        name: `Grade-Type-${i}`,
        deleteable: false,
        editable: true,
        description: 'Some descriptor string'
    });
}

let placeholders = [];
for (let i = 0; i < NUM_PLACEHOLDERS; i++) {
    placeholders.push({
        type: 'node.placeholder.',
        uuid: `placeholder-js-${i}`,
        name: `Plaeholder-${i}`,
        deleteable: false,
        editable: true,
        description: 'Some descriptor string',

        pending_node: {
            type: 'node.pose.thing.',
            uuid: `thing-js-${i}`,
            name: `Thing-${i}`,
            deleteable: true,
            editable: true,
            description: 'Some descriptor string (optional)', // could be ''

            thing_type_uuid: thingTypes[0].uuid, // gets a thing_type from other fake data
            position: '<pending>',
            orientation: '<pending>'
        },
        pending_fields: [
            'position',
            'orientation'
        ]
    });
}

let things = [];
for (let i = 0; i < NUM_THINGS; i++) {
    let t = {
        type: 'node.pose.thing.',
        uuid: `thing-js-${i}`,
        name: `Thing-${i}`,
        deleteable: true,
        editable: true,
        description: 'Some descriptor string (optional)', // could be ''

        link: '', //'' means default to world or app
        thing_type_uuid: thingTypes[0].uuid, // gets a thing_type from other fake data
        position: {
            type: 'node.position.',
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
            type: 'node.orientation.',
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
    };

    things.push(t);
    placeholders.push({
        type: 'node.placeholder.',
        uuid: `placeholder-js-${NUM_PLACEHOLDERS+i}`,
        name: `Plaeholder-${NUM_PLACEHOLDERS+i}`,
        deleteable: false,
        editable: true,
        description: 'Some descriptor string',

        pending_node: t,
        pending_fields: []
    })
}

/*****************************************************************
*  Positional Data
* - Waypoint
* - Location
* - Thing (instances)
*****************************************************************/
//onChange={e=>setItemProperty(focusItem.type,focusItem.uuid,'name',e.target.value)}/>
let waypoints = [];
for (let i = 0; i < NUM_WAYPOINTS; i++) {
    const num = Math.random();
    waypoints.push({
        type: 'node.pose.waypoint.',
        uuid: `waypoint-js-${i}`,
        name: `Waypoint-${i}`,
        deleteable: true,
        editable: true,
        description: 'Some descriptor string (optional)', // could be ''

        link: '', //'' means default to world or app
        joints: {
            type: 'node.joints.',
            uuid: `joints-js-waypoint-${i}`,
            name: '',
            deleteable: false,
            editable: true,
            description: 'Some descriptor string (optional)', // could be ''

            joint_positions: [0, 0, 0], // or null
            joint_names: ['j1', 'j2', 'j3'], // or null
            reachable: (num > 0.5 ? true : false), // better than having to check the array to generate the flag
            length: 3 // this is enforced on the backend for positions and names
        },
        position: {
            type: 'node.position.',
            uuid: `position-js-waypoint-${i}`,
            name: '',
            deleteable: false,
            editable: true,
            description: 'Some descriptor string (optional)', // could be ''

            x: .5*(Math.random()-0.5),
            y: .5*(Math.random()-0.5),
            z: .5*(Math.random()-0.5)+0.25
        },
        orientation: {
            type: 'node.orientation.',
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
for (let i = 0; i < NUM_LOCATIONS; i++) {
    const num = Math.random();
    locations.push({
        type: 'node.pose.waypoint.location.',
        uuid: `location-js-${i}`,
        name: `Location-${i}`,
        deleteable: !(i === 3),
        editable: !(i % 4 === 0),
        description: 'Some descriptor string (optional)', // could be ''

        link: '', //'' means default to world or app
        joints: {
            type: 'node.joints.',
            uuid: `joints-js-location-${i}`,
            name: '',
            deleteable: false,
            editable: true,
            description: 'Some descriptor string (optional)', // could be ''

            joint_positions: [0, 0, 0], // or null
            joint_names: ['j1', 'j2', 'j3'], // or null
            reachable:(num > 0.5 ? true : false), // better than having to check the array to generate the flag
            length: 3 // this is enforced on the backend for positions and names
        },
        position: {
            type: 'node.position.',
            uuid: `position-js-location-${i}`,
            name: '',
            deleteable: false,
            editable: true,
            description: 'Some descriptor string (optional)', // could be ''

            x: .5*(Math.random()-0.5),
            y: .5*(Math.random()-0.5),
            z: .5*(Math.random()-0.5)+0.25
        },
        orientation: {
            type: 'node.orientation.',
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



//NOTE: Andy is simmering (but like not upset) in regards to shadow things


/*****************************************************************
* Regions
*****************************************************************/

// Probably don't visualize all of fields of regions
const generic_region = {
    type: 'node.pose.region.',
    uuid: `region-js-0`,
    name: `Region-0`,
    deleteable: true,
    editable: true,
    description: 'Some descriptor string (optional)', // could be ''

    link: '', //'' means default to world or app
    // position is known in a default region, just uncertainty in orientation
    center_position: {
        type: 'node.position.',
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
        type: 'node.orientation.',
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
        type: 'node.orientation.',
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
    type: 'node.pose.region.cube-region.',
    uuid: `cube-region-js-1`,
    name: `Cube-Region-1`,
    deleteable: false,
    editable: false,
    description: 'Some descriptor string (optional)', // could be ''

    link: '', //'' means default to world or app
    center_position: {
        type: 'node.position.',
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
        type: 'node.orientation.',
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

    link: '', //'' means default to world or app
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

const machine_generator = {
    type: 'node.machine.',
    uuid: `machine-js-generator`,
    name: `Machine-Generator`,
    deleteable: false,
    editable: false,
    description: 'Some descriptor string (optional)', // could be ''

    process_time: 5, //sec
    inputs: {}

    , // make this a tuple as value of keys

    outputs: {
        'thing-type-js-0': [
            {
                region_uuid: cube_region.uuid,
                quantity: 1,
                placeholder_uuids: ['placeholder-js-0']
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

        link: '', //'' means default to world or app
        position: {
            type: 'node.position.',
            uuid: `position-js-machine-0`,
            name: `Position-0`,
            deleteable: false,
            editable: false,
            description: 'Some descriptor string (optional)', // could be ''

            x: 0,
            y: 0,
            z: 0
        },
        orientation: {
            type: 'node.orientation.',
            uuid: `orientation-js-machine-0`,
            name: `Orientation-0`,
            deleteable: false,
            editable: false,
            description: 'Some descriptor string (optional)', // could be ''

            x: 0,
            y: 0,
            z: 0,
            w: 1
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
    outputs: {},
    mesh_id: 'package:/app/meshes/3d_printer.fbx',
    pose_offset: { // Local transform
        type: 'node.pose.',
        uuid: `pose-js-machine-1`,
        name: `Pose-1`,
        deleteable: false,
        editable: false,
        description: 'Some descriptor string (optional)', // could be ''

        link: '', //'' means default to world or app
        position: {
            type: 'node.position.',
            uuid: `position-js-machine-1`,
            name: `Position-1`,
            deleteable: false,
            editable: false,
            description: 'Some descriptor string (optional)', // could be ''

            x: 0,
            y: 0,
            z: 0
        },
        orientation: {
            type: 'node.orientation.',
            uuid: `orientation-js-machine-1`,
            name: `Orientation-1`,
            deleteable: false,
            editable: false,
            description: 'Some descriptor string (optional)', // could be ''

            x: 0,
            y: 0,
            z: 0,
            w: 1
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
            },
            {
                region_uuid: sphere_region.uuid,
                quantity: 2
            }
        ]
    },
    outputs: {
        'thing-type-js-1': [
            {
                region_uuid: sphere_region.uuid,
                quantity: 1,
                placeholder_uuids: ['placeholder-js-1']
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

        link: '', //'' means default to world or app
        position: {
            type: 'node.position.',
            uuid: `position-js-machine-2`,
            name: `Position-2`,
            deleteable: false,
            editable: false,
            description: 'Some descriptor string (optional)', // could be ''

            x: 0,
            y: 0,
            z: 0
        },
        orientation: {
            type: 'node.orientation.',
            uuid: `orientation-js-machine-2`,
            name: `Orientation-2`,
            deleteable: false,
            editable: false,
            description: 'Some descriptor string (optional)', // could be ''

            x: 0,
            y: 0,
            z: 0,
            w: 1
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
*   - Trace
*****************************************************************/

let trajectories = [];
for (let i = 0; i < NUM_TRAJECTORIES; i++) {
    trajectories.push({
        type: 'node.trajectory.',
        uuid: `trajectory-js-${i}`,
        name: `Trajectory-${i}`,
        deleteable: false,
        editable: true,
        description: 'Some descriptor string (optional)', // could be ''

        start_location_uuid: locations[0].uuid,
        end_location_uuid: locations[1].uuid,
        waypoint_uuids: [
            // presumably filled with arbitrary (ordered) list of waypoint uuids
            `waypoint-js-${i*2}`,
            `waypoint-js-${i*2+1}`,
            `waypoint-js-${i*2+2}`
        ],
        trace: { // trace can be null, supplied by backend when planning is complete
            type: 'node.trace.',
            uuid: `trace-js-trajectory-${i}`,
            name: `Trace-${i}`,
            deleteable: false,
            editable: false,
            description: 'Some descriptor string (optional)', // could be ''

            time_data: [0, /*...*/], // each timestep, time from relative start
            joint_names: ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'], // name for each joint corresponding to inner array on joint_data
            joint_data: [[0, 0, 0, 0, 0, 0]], // joint state of robot at each timestep (NOTE that this is not an EvD Joints just a simple list)
            tf_data: {
                'ee_link': [ // Pose at that frame at each timestep (NOTE that this is not an EvD Pose just a simple dict)
                    {
                        position: { x: 0, y: 0, z: 0 },
                        orientation: { x: 0, y: 0, z: 0, w: 0 },
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
            joints_paths: ['joint_tf_frame_1', 'joint_tf_frame_2'], // tf-frames of arm joints being tracked
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
    offset: { // assume frame is robot's root
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
for (let i = 0; i < NUM_COLLISION_MESHES; i++) {
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

            link: '', //'' means default to world or app
            position: {
                type: 'node.position.',
                uuid: `position-js-collision-mesh-${i}`,
                name: `Position-${i}`,
                deleteable: false,
                editable: false,
                description: 'Some descriptor string (optional)', // could be ''

                x: 0,
                y: 0,
                z: 0
            },
            orientation: {
                type: 'node.orientation.',
                uuid: `orientation-js-collision-mesh-${i}`,
                name: `Orientation-${i}`,
                deleteable: false,
                editable: false,
                description: 'Some descriptor string (optional)', // could be ''

                x: 0,
                y: 0,
                z: 0,
                w: 1
            }
        },
        link: 'some_frame' // Can be used in the TF tree as a frame (could be '' or 'app' which means 'world')
    });
}

let occupancyZones = [ // assume frame is app / world
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
        scale_z: 0.5,  // 'radius' (e.g. total width is 2x this)
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
        scale_z: 0.5,  // 'radius' (e.g. total width is 2x this)
        height: 0 // typically a negative value (wherever ground is)
    }
];

let pinchPoints = [];
for (let i = 0; i < NUM_PINCH_POINTS; i++) {
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

            x: 0,
            y: 0,
            z: 0
        },
        link: 'some_robot_frame',
        radius: 0.25,
        length: 0.5
    });
}

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
    type: 'node.primitive.hierarchical.program.',
    uuid: `program-js`,
    name: `Test Program`,
    deleteable: false,
    editable: true,
    description: 'Some descriptor string (optional)', // could be ''

    skills: [
        {
            type: 'node.primitive.hierarchical.skill.',
            name: 'Machine Blocking Process',
            uuid: 'machine-blocking-process-skill-uuid',
            editable: false,
            deleteable: false,
            description: '',
            parameters: {},
            arguments: [
                {
                    type: 'node.skill-argument.',
                    // The human-readable name of this argument
                    name: 'This Machine',
                    // If the uuid is found in any fields for the children primitives' parameters, replace it with the corresponding value. 
                    // This also serves as the key for any corresponding skill-call's parameters.
                    uuid: 'skill-arg-uuid-0', 
                    editable: true,
                    deleteable: true,
                    description: '',
                    // The type of the argument
                    parameter_type: 'node.machine.',
                    is_list: false
                }
            ],
            primitives: [
                {
                    type: 'node.primitive.machine-primitive.machine-start.',
                    name: 'Machine Start',
                    uuid: 'skill-machine-start-uuid',
                    editable: true,
                    deleteable: false,
                    description: '',
                    parameters: {
                        machine_uuid: 'skill-arg-uuid-0'
                    }
                },
                {
                    type: 'node.primitive.machine-primitive.machine-wait.',
                    name: 'Machine Wait',
                    uuid: 'skill-machine-wait-uuid-0',
                    editable: true,
                    deleteable: false,
                    description: '',
                    parameters: {
                        machine_uuid: 'skill-arg-uuid-0'
                    }
                },
                {
                    type: 'node.primitive.machine-primitive.machine-wait.',
                    name: 'Machine Wait',
                    uuid: 'skill-machine-wait-uuid-1',
                    editable: true,
                    deleteable: false,
                    description: '',
                    parameters: {
                        machine_uuid: 'skill-arg-uuid-0'
                    }
                },
            ]
        },
        {
            type: 'node.primitive.hierarchical.skill.',
            name: 'Simple Pick and Place',
            uuid: 'simple-pick-and-place-uuid',
            editable: false,
            deleteable: false,
            description: '',
            parameters: {},
            arguments: [
                {
                    type: 'node.skill-argument.',
                    name: 'Pick Trajectory',
                    uuid: 'skill-arg-uuid-1',
                    editable: false,
                    deleteable: false,
                    description: '',
                    parameter_type: 'node.trajectory.',
                    is_list: false
                },
                {
                    type: 'node.skill-argument.',
                    name: 'Place Trajectory',
                    uuid: 'skill-arg-uuid-2',
                    editable: false,
                    deleteable: false,
                    description: '',
                    parameter_type: 'node.trajectory.',
                    is_list: false
                },
                {
                    type: 'node.skill-argument.',
                    name: 'Thing to Pick',
                    uuid: 'skill-arg-uuid-3',
                    editable: false,
                    deleteable: false,
                    description: '',
                    parameter_type: 'node.pose.thing.',
                    is_list: false
                }
            ],
            primitives: [
                {
                    type: 'node.primitive.move-trajectory.',
                    uuid: 'move-trajectory-uuid-2',
                    name: 'Move Trajectory',
                    editable: false,
                    deleteable: false,
                    description: '',
                    parameters: {
                        manual_safety: false,
                        trajectory_uuid: 'skill-arg-uuid-1'
                    }
                },
                {
                    type: 'node.primitive.gripper.',
                    uuid: 'gripper-grasp-uuid-2',
                    name: 'Grasp',
                    editable: true,
                    deleteable: false,
                    description: '',
                    parameters: {
                        position: 0,
                        speed: 1,
                        effort: 1,
                        semantic: 'grasping',
                        thing_uuid: 'skill-arg-uuid-3'
                    }
                },
                {
                    type: 'node.primitive.move-trajectory.',
                    uuid: 'move-trajectory-uuid-3',
                    name: 'Move Trajectory',
                    editable: false,
                    deleteable: false,
                    description: '',
                    parameters: {
                        manual_safety: false,
                        trajectory_uuid: 'skill-arg-uuid-2'
                    }
                },
                {
                    type: 'node.primitive.gripper.',
                    uuid: 'gripper-grasp-uuid-3',
                    name: 'Release',
                    editable: true,
                    deleteable: true,
                    description: '',
                    parameters: {
                        position: 0,
                        speed: 1,
                        effort: 1,
                        semantic: 'releasing',
                        thing_uuid: 'skill-arg-uuid-3'
                    }
                }
            ]
        },
        {
            type: 'node.primitive.hierarchical.skill.',
            name: 'Initialize',
            uuid: 'initialize-skill-uuid',
            editable: false,
            deleteable: false,
            parameters: {},
            arguments: [ 
                {
                    type: 'node.skill-argument.',
                    name: 'Machine to Start', // Human-readable name for this variable
                    uuid: 'skill-arg-uuid-4', // Use this value as the parameter key in the skill-call that calls this skill.
                    editable: false,
                    deleteable: false,
                    description: '',
                    parameter_type: 'node.machine.', // The enforced type of whatever node object this argument would refer to.
                    is_list: false
                },
                {
                    type: 'node.skill-argument.',
                    name: 'Starting Location',
                    uuid: 'skill-arg-uuid-5',
                    editable: false,
                    deleteable: false,
                    description: '',
                    parameter_type: 'node.pose.waypoint.location.',
                    is_list: false
                }
            ],
            primitives: [
                { // This is only for display purposes. what really happens in that this
                    // gets expanded for each machine at runtime
                    type: 'node.primitive.machine-primitive.machine-initialize.',
                    uuid: 'machine-init-uuid-0',
                    name: 'Initialize Machine',
                    deleteable: false,
                    editable: true,
                    description: '',
                    parameters: {
                        machine_uuid: 'skill-arg-uuid-4'
                    }
                },
                {
                    type: 'node.primitive.gripper.',
                    uuid: 'gripper-grasp-uuid-5',
                    name: 'Reset Gripper',
                    editable: true,
                    deleteable: false,
                    description: '',
                    parameters: {
                        position: 0,
                        speed: 1,
                        effort: 1,
                        semantic: 'ambiguous',
                        thing_uuid: null
                    }
                },
                {
                    type: 'node.primitive.move-unplanned.',
                    uuid: 'move-unplanned-uuid-0',
                    name: 'Move Unplanned',
                    deleteable: false,
                    editable: true,
                    description: '',
                    parameters: {
                        manual_safety: false,
                        velocity: 1,
                        move_type: 'joint',
                        location_uuid: 'skill-arg-uuid-5'
                    }
                }
            ]
        },
        {
            type: 'node.primitive.hierarchical.skill.',
            uuid: 'open-gripper-skill-uuid',
            name: 'Release',
            editable: false,
            deleteable: false,
            description: '',
            parameters: {},
            arguments: [
                {
                    type: 'node.skill-argument.',
                    uuid: 'skill-arg-uuid-6',
                    name: 'Thing to Release',
                    editable: false,
                    deleteable: false,
                    description: '',
                    parameter_type: 'node.pose.thing.',
                    is_list: false
                }
            ],
            primitives: [
                {
                    type: 'node.primitive.gripper.',
                    uuid: 'gripper-grasp-uuid-6',
                    name: 'Open Gripper',
                    editable: true,
                    deleteable: false,
                    description: '',
                    parameters: {
                        position: 100,
                        speed: 1,
                        effort: 1,
                        semantic: 'releasing',
                        thing_uuid: 'skill-arg-uuid-6'
                    }
                }
            ]
        },
        {
            type: 'node.primitive.hierarchical.skill.',
            uuid: 'close-gripper-skill-uuid',
            name: 'Close Gripper',
            editable: false,
            deleteable: false,
            description: '',
            parameters: {},
            arguments: [
                {
                    type: 'node.skill-argument.',
                    uuid: 'skill-arg-uuid-7',
                    name: 'Thing to Grasp',
                    editable: false,
                    deleteable: false,
                    description: '',
                    parameter_type: 'node.pose.thing.',
                    is_list: false
                }
            ],
            primitives: [
                {
                    type: 'node.primitive.gripper.',
                    uuid: 'gripper-grasp-uuid-7',
                    name: 'Grasp',
                    editable: true,
                    deleteable: false,
                    description: '',
                    parameters: {
                        position: 0,
                        speed: 1,
                        effort: 1,
                        semantic: 'grasping',
                        thing_uuid: 'skill-arg-uuid-7'
                    }
                }
            ]
        }
    ],

    environment: {
        reach_sphere: reachSphere,
        pinch_points: pinchPoints,
        collision_meshes: collisionMeshes,
        occupancy_zones: occupancyZones,
        locations: locations,
        machines: machines,
        waypoints: waypoints,
        trajectories: trajectories,
        thing_types: thingTypes,
        regions: regions,
        grade_types: gradeTypes,
        placeholders: placeholders
    },

    primitives: [ // just initializes but that is enough to get the point across >(0_0)<
        {
            type: 'node.primitive.skill-call.',
            uuid: 'some-skill-call-uuid',
            name: 'Execute Skill',
            editable: true,
            deleteable: true,
            description: '',
            parameters: {
                skill_uuid: 'initialize-skill-uuid',
                'skill-arg-uuid-4': 'machine-js-transformer',
                'skill-arg-uuid-5': 'location-js-5'
            }
        },

        {
            type: 'node.primitive.move-trajectory.',
            uuid: 'move-trajectory-uuid-0',
            name: 'Move Trajectory',
            editable: false,
            deleteable: false,
            description: '',
            parameters: {
                manual_safety: false,
                trajectory_uuid: 'trajectory-js-0'
            }
        },
        {
            type: 'node.primitive.gripper.',
            uuid: 'gripper-grasp-uuid-0',
            name: 'Grasp',
            editable: true,
            deleteable: true,
            description: '',
            parameters: {
                position: 0,
                speed: 1,
                effort: 1,
                semantic: 'grasping',
                thing_uuid: 'placeholder-js-0'
            }
        },
        {
            type: 'node.primitive.move-trajectory.',
            uuid: 'move-trajectory-uuid-1',
            name: 'Move Trajectory',
            editable: false,
            deleteable: false,
            description: '',
            parameters: {
                manual_safety: false,
                trajectory_uuid: 'trajectory-js-1'
            }
        },
        {
            type: 'node.primitive.gripper.',
            uuid: 'gripper-grasp-uuid-1',
            name: 'Release',
            editable: true,
            deleteable: false,
            description: '',
            parameters: {
                position: 0,
                speed: 1,
                effort: 1,
                semantic: 'releasing',
                thing_uuid: 'placeholder-js-0'
            }
        }

    ]
};


//=================================================================

/// Export
const fields = {

    // all of these in environment
    waypoints,
    locations,
    machines,
    thingTypes,
    gradeTypes,
    regions,
    trajectories,
    reachSphere,
    collisionMeshes,
    occupancyZones,
    pinchPoints,
    placeholders,

    // evd_program
    program
};

export default fields;
