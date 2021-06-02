/*
 * Fake data attempts to expess EvDscript's datastructure without
 * need for ROS connection. Useful for development and expectation
 * documentation.
 */

const NUM_THING_TYPES = 5;
const NUM_WAYPOINTS = 20;
const NUM_LOCATIONS = 20;
const NUM_GENERIC_REGIONS = 5;
const NUM_CUBE_REGIONS = 2;
const NUM_SPHERE_REGIONS = 2;
const NUM_THINGS = 10;
const NUM_MACHINES = 3;
const NUM_TRAJECTORIES = 4;
const NUM_COLLISION_MESHES = 2;
const NUM_OCCUPANCY_ZONES = 0;
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
        mesh_id: 'default.stl',
        is_safe: true,
        weight: 1.0
    });
}

/***************************************************************** 
*  Positional Data
* - Waypoint
* - Location
* - Region
*   - CubeRegion
*   - SphereRegion
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

let regions = [];
for (let i=0; i<NUM_GENERIC_REGIONS; i++) {
    regions.push({
        uuid: `region-js-${i}`,
        name: `Region-${i}`,
        deleteable: i % 3,
        editable: i % 3,

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

        mesh: 'default.stl',
        input_regions: {},
        output_regions: {}
    });
}

/***************************************************************** 
* Trajectories
* - Trajectory
* - Trace
* - TraceDataPoint
* - Grade
*****************************************************************/

let trajectories = [];
for (let i=0; i<NUM_TRAJECTORIES; i++) {

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

}

let occupancyZones = [];
for (let i=0; i<NUM_OCCUPANCY_ZONES; i++) {

}

let pinchPoints = [];
for (let i=0; i<NUM_PINCH_POINTS; i++) {

}


/// Export
const fields = {
    waypoints,
    regions,
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