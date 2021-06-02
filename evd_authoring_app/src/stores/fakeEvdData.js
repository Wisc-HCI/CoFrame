/*
 * Fake data attempts to expess EvDscript's datastructure without
 * need for ROS connection. Useful for development and expectation
 * documentation.
 */

const NUM_THING_TYPES = 5;
const NUM_WAYPOINTS = 20;
const NUM_LOCATIONS = 20;
const NUM_THINGS = 10;
const NUM_MACHINES = 3;
const NUM_TRAJECTORIES = 4;
const NUM_COLLISION_MESHES = 2;
const NUM_PINCH_POINTS = 0;


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

        type_name: `Thing-Type-${i}`,
        mesh_id: 'package://app/meshes/3DBenchy.stl', // This specific pathing is subject to change (using robot-scene variant for now)
        is_safe: true,  // Checklist that creates this aggregate value
        weight: 1.0 // kg in one g (evd will not work natively on the moon)
        // Assume scale is (1,1,1) - meshes sized correctly
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
* Machines
* - Machine
* - Machine Recipe
* - ThingType & Region Mapping
*****************************************************************/

let machines = [];
for (let i=0; i<NUM_MACHINES; i++) {
    machines.push({
        uuid: `machine-js-${i}`,
        name: `Machine-${i}`,
        deleteable: i % 3,
        editable: i % 2,

        mesh_id: 'default.stl',
        input_regions: {}, // make this a tuple as value of keys
        output_regions: {}
    });
}

/*
// Probably don't visualize all of fields of regions
let regions = [];
for (let i=0; i<NUM_GENERIC_REGIONS; i++) {
    regions.push({
        uuid: `region-js-${i}`,
        name: `Region-${i}`,
        deleteable: i % 3,
        editable: i % 3,

        // position is known in a default region, just uncertainty in orientation
        center_position: {
            uuid: `position-js-region-${i}`,
            name: '',
            deleteable: false,
            editable: true,

            x: 0,
            y: 0,
            z: 0
        },
        center_orientation: {
            uuid: `orientation-js-region-${i}`,
            name: '',
            deleteable: false,
            editable: true,

            x: 0,
            y: 0,
            z: 0,
            w: 1
        },
        free_orientation: (i % 2) ? true : false,
        uncertainty_orientation_limit: 1.0,
        // if free_orientation then it is null (don't care case)
        // if not free_orientation it can be null meaning use the center_orientation as target
            // target relative to other node for qaternion distance
        // otherwise switch over to alt target as the relative node for distance calc
        uncertainty_orientation_alt_target: (i % 2) ? null : {
            uuid: `orientation-js-region-alt-${i}`,
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

// Set to free orientation for simplicity only - can have orientation constraints
for (let i=0; i<NUM_CUBE_REGIONS; i++) {
    regions.push({
        uuid: `cube-region-js-${i}`,
        name: `Cube-Region-${i}`,
        deleteable: i % 3,
        editable: i % 3,

        center_position: {
            uuid: `position-js-cube-region-${i}`,
            name: '',
            deleteable: false,
            editable: true,

            x: 0,
            y: 0,
            z: 0
        },
        center_orientation: {
            uuid: `orientation-js-cube-region-${i}`,
            name: '',
            deleteable: false,
            editable: true,

            x: 0,
            y: 0,
            z: 0,
            w: 1
        },
        free_orientation: true,
        uncertainty_orientation_limit: 1.0,
        uncertainty_orientation_alt_target: null,
        uncertainty_x: 1.0,
        uncertainty_y: 1.0,
        uncertainty_z: 1.0
    });
}

// Set to free orientation for simplicity only - can have orientation constraints
for (let i=0; i<NUM_SPHERE_REGIONS; i++) {
    regions.push({
        uuid: `sphere-region-js-${i}`,
        name: `Sphere-Region-${i}`,
        deleteable: i % 3,
        editable: i % 3,

        center_position: {
            uuid: `position-js-sphere-region-${i}`,
            name: '',
            deleteable: false,
            editable: true,

            x: 0,
            y: 0,
            z: 0
        },
        center_orientation: {
            uuid: `orientation-js-sphere-region-${i}`,
            name: '',
            deleteable: false,
            editable: true,

            x: 0,
            y: 0,
            z: 0,
            w: 1
        },
        free_orientation: true,
        uncertainty_orientation_limit: 1.0,
        uncertainty_orientation_alt_target: null,
        uncertainty_radius: 1.0
    });
}
*/

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

            data: {
                'ee_link': [ // TraceDatePoint
                    {
                        uuid: `trace-data-point-ee_link-trace-js-trajectory-${i}`,
                        name: `TraceDataPoint-${i}`,
                        deleteable: false,
                        editable: false,

                        // NOTE: have predefined grade types: ENUM?
                        grades: {
                            'grade_max_velocity': {
                                uuid: `grade-trace-data-point-ee_link-trace-js-trajectory-${i}`,
                                name: `Grade-${i}`,
                                deleteable: false,
                                editable: false,

                                grade_type: 'grade_max_velocity',
                                value: 0.5 // 0 to 1 (all grades) though the semantics of that is dependent on the grader
                            },
                            //... (e.g. grade_min_velocity, collision_proximity)
                        },
                        position: {/*...*/},
                        orientation: {/*...*/},
                    }
                ],
                // ... (e.g keys for joint_tf_frame_1, gripper_tf_frame_1)
            },
            time: 1, // sec
            end_effector_path: 'ee_link', //the frame of the end-effector,
            joints_paths: ['joint_tf_frame_1','joint_tf_frame_2'], // tf-frames of arm joints being tracked
            tool_paths: ['gripper_tf_frame_1'], // tf-frames for grippers
            component_paths: ['thing_tf_frame'], // (optional / may no longer be needed)
        },
        velocity: 0.5, // suggestion (user configured)
        move_type: 'joint' // or 'ee_ik'

        // There will probably be a joint_angles property in trace which will be a list of joint states corresponding to the data list
        // List of times through the trace as additional field ("Index" vector)
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
        uuid: `pose-js-reach-sphere-offset-0`,
        name: '',
        deleteable: false,
        editable: false,

        position: {
            uuid: `position-js-reach-sphere-offset-0`,
            name: '',
            deleteable: false,
            editable: true,

            x: 0,
            y: 0,
            z: 0
        },
        orientation: {
            uuid: `orientation-js-reach-sphere-offset-0`,
            name: '',
            deleteable: false,
            editable: true,

            x: 0,
            y: 0,
            z: 0,
            w: 1
        }
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
    waypoints,
    locations,
    machines,
    thingTypes,
    things,
    trajectories,
    reachSphere,
    collisionMeshes,
    occupancyZones,
    pinchPoints
};

export default fields;