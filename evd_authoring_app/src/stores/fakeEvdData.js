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

        joints: (i % 3) ? (null) : ((i % 5) ? [0,0,0,0,0,0] : []), // variantions on joint information (dependent on reachability)
        // if null then needs to run grader
        // if length === 0 then failed to reach
        // if length > 0 filled with joint info (indexed in default order from base_link) - assumes single-arm
        position: {
            uuid: `position-js-waypoint-${i}`,
            name: '',
            deleteable: false,
            editable: true,

            x: 0,
            y: 0,
            z: 0
        },
        orientation: {
            uuid: `orientation-js-waypoint-${i}`,
            name: '',
            deleteable: false,
            editable: true,

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

        joints: (i % 3) ? (null) : ((i % 5) ? [0,0,0,0,0,0] : []), // variantions on joint information (dependent on reachability)
        position: {
            uuid: `position-js-location-${i}`,
            name: '',
            deleteable: false,
            editable: true,

            x: 0,
            y: 0,
            z: 0
        },
        orientation: {
            uuid: `orientation-js-location-${i}`,
            name: '',
            deleteable: false,
            editable: true,

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

        thing_type_uuid: thingTypes[0].uuid, // gets a thing_type from other fake data
        position: {
            uuid: `position-js-thing-${i}`,
            name: '',
            deleteable: false,
            editable: true,

            x: 0,
            y: 0,
            z: 0
        },
        orientation: {
            uuid: `orientation-js-thing-${i}`,
            name: '',
            deleteable: false,
            editable: true,

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

    // position is known in a default region, just uncertainty in orientation
    center_position: {
        uuid: `position-js-region-0`,
        name: '',
        deleteable: false,
        editable: true,

        x: 0,
        y: 0,
        z: 0
    },
    center_orientation: {
        uuid: `orientation-js-region-0`,
        name: '',
        deleteable: false,
        editable: true,

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

    center_position: {
        uuid: `position-js-cube-region-1`,
        name: '',
        deleteable: false,
        editable: true,

        x: 0,
        y: 0,
        z: 0
    },
    center_orientation: {
        uuid: `orientation-js-cube-region-1`,
        name: '',
        deleteable: false,
        editable: true,

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
    uuid: `sphere-region-js-2`,
    name: `Sphere-Region-2`,
    deleteable: true,
    editable: false,

    center_position: {
        uuid: `position-js-sphere-region-3`,
        name: '',
        deleteable: false,
        editable: true,

        x: 0,
        y: 0,
        z: 0
    },
    center_orientation: {
        uuid: `orientation-js-sphere-region-3`,
        name: '',
        deleteable: false,
        editable: true,

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
    uuid: `machine-js-generator`,
    name: `Machine-Generator`,
    deleteable: false,
    editable: false,

    process_time: 5, //sec
    inputs: {}, // make this a tuple as value of keys
    outputs: {
        'thing-type-js-0': [
            {
                region_uuid: cube_region.uuid,
                quantity: 1
            }
        ]
    }
}

const machine_consumer = {
    uuid: `machine-js-consumer`,
    name: `Machine-Consumer`,
    deleteable: false,
    editable: false,

    process_time: 2, //sec
    inputs: {
        'thing-type-js-1': [
            {
                region_uuid: generic_region.uuid,
                quantity: 1
            }
        ]
    }, 
    outputs: { }
};

const machine_transformer = {
    uuid: `machine-js-transformer`,
    name: `Machine-Transformer`,
    deleteable: false,
    editable: false,

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
    }
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

let trajectories = [];
for (let i=0; i<NUM_TRAJECTORIES; i++) {
    trajectories.push({
        uuid: `trajectory-js-${i}`,
        name: `Trajectory-${i}`,
        deleteable: false,
        editable: false,

        start_location_uuid: locations[0].uuid,
        end_location_uuid: locations[1].uuid,
        waypoint_uuids: [
            // presumably filled with arbitrary (ordered) list of waypoint uuids
        ],
        trace: {
            uuid: `trace-js-trajectory-${i}`,
            name: `Trace-${i}`,
            deleteable: false,
            editable: false,

            time_data: [0], // each timestep, time from relative start
            joint_data: [[0,0,0,0,0,0]], // joint state of robot at each timestep 
            tf_data: {
                'ee_link': [ // Pose at that frame at each timestep
                    {
                        uuid: `pose-ee_link-trace-js-trajectory-${i}`,
                        name: `TraceDataPoint-${i}`,
                        deleteable: false,
                        editable: false,

                        position: {/*...*/},
                        orientation: {/*...*/},
                    }
                ],
                // ... (e.g keys for joint_tf_frame_1, gripper_tf_frame_1)
            },
            // Keys are predefined grade types
            grades: {
                'grade_max_velocity': [0.5] // list of grade values from 0 to 1 (for all grades), though semantics of that is dependent on the grader
                //... (e.g. grade_min_velocity, collision_proximity)
            },

            // length of time_data, joint_data, each value list in tf_data and in graders should be equal

            time: 1, // sec
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
    uuid: `reach-sphere-js-0`,
    name: `ReachSphere`,
    deleteable: false,
    editable: false,

    radius: 1.0,
    offset: {
        uuid: `position-js-reach-sphere-offset-0`,
        name: '',
        deleteable: false,
        editable: true,

        x: 0,
        y: 0,
        z: 0
    }
};

let collisionMeshes = [];
for (let i=0; i<NUM_COLLISION_MESHES; i++) {
    collisionMeshes.push({
        uuid: `collision-mesh-js-${i}`,
        name: `CollisionMesh-${i}`,
        deleteable: false,
        editable: false,

        mesh_id: 'default.stl',
        pose_offset: { // Local transform
            uuid: `pose-js-collision-mesh-${i}`,
            name: `Pose-${i}`,
            deleteable: false,
            editable: false,

            position: {
                uuid: `position-js-collision-mesh-${i}`,
                name: `Position-${i}`,
                deleteable: false,
                editable: false,

                x:0,
                y:0,
                z:0
            },
            orientation: {
                uuid: `orientation-js-collision-mesh-${i}`,
                name: `Orientation-${i}`,
                deleteable: false,
                editable: false,

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
        uuid: `occupancy-zone-js-0`,
        name: `Human Occupancy Zone`,
        deleteable: false,
        editable: false,

        occupancy_type: 'human', // human is conceptualized as a box
        position_x: 0, // 'center'
        position_z: 0, // 'center'
        scale_x: 1,    // 'radius' (e.g. total width is 2x this)
        scale_y: 0.5,  // 'radius' (e.g. total width is 2x this)
        height: 0 // typically a negative value (wherever ground is)
    },
    {
        uuid: `occupancy-zone-js-1`,
        name: `Human Occupancy Zone`,
        deleteable: false,
        editable: false,

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
        uuid: `pinch-points-js-${i}`,
        name: `Pinch Point ${i}`,
        deleteable: false,
        editable: false,

        // conceptualized as a cylinder
        axis: 'x', // 'x', 'y', or 'z' ~ aligned along one of these axes
        offset: { // positional offset from link
            uuid: `position-js-pinch-point-${i}`,
            name: `Position-${i}`,
            deleteable: false,
            editable: false,

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
* - Error Blob
*****************************************************************/

// TODO

/***************************************************************** 
* Issue Server
* - Issues
* - Pending Jobs
*****************************************************************/

// TODO

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

// TODO

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
    pinchPoints
};

export default fields;