import { generateUuid } from "../generateUuid"

export const findReachabilityIssues = (program) => { // requires joint_processor to produce joints for each waypoint/location
    let issues = {};
    let usedPoses = [];
    // First check through the trajectories that are used.
    Object.values(program.data.trajectories).forEach(trajectory=>{
        // Check through the start locations. If not defined, this is resolved in program quality.
        if (trajectory.start_location_uuid) {
            const startLocation = program.data.locations[trajectory.start_location_uuid];
            if (!startLocation.joints.reachable && usedPoses.indexOf(trajectory.start_location_uuid) < 0) {
                usedPoses.push(trajectory.start_location_uuid)
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    uuid: uuid,
                    requiresChanges: true,
                    title: `Location "${startLocation.name}" not reachable`,
                    description: `Location "${startLocation.name}" is not reachable by the robot, but is used in the program.`,
                    complete: false,
                    focus: {uuid:trajectory.start_location_uuid, type:'location'},
                    graphData: null
                }
            }
        };
        // Check through the end locations. If not defined, this is resolved in program quality.
        if (trajectory.end_location_uuid) {
            const endLocation = program.data.locations[trajectory.end_location_uuid];
            if (!endLocation.joints.reachable && usedPoses.indexOf(trajectory.end_location_uuid) < 0) {
                usedPoses.push(trajectory.end_location_uuid)
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    uuid: uuid,
                    requiresChanges: true,
                    title: `Location "${endLocation.name}" not reachable`,
                    description: `Location "${endLocation.name}" is not reachable by the robot, but is used in the program.`,
                    complete: false,
                    focus: {uuid:trajectory.end_location_uuid, type:'location'},
                    graphData: null
                }
            }
        }
        // Now go through the waypoints of the trajectory
        trajectory.waypoint_uuids.forEach(waypoint_uuid=>{
            const waypoint = program.data.waypoints[waypoint_uuid];
            if (!waypoint.joints.reachable && usedPoses.indexOf(waypoint_uuid) < 0) {
                usedPoses.push(waypoint_uuid)
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    uuid: uuid,
                    requiresChanges: true,
                    title: `Waypoint "${waypoint.name}" not reachable`,
                    description: `Waypoint "${waypoint.name}" is not reachable by the robot, but is used in the program.`,
                    complete: false,
                    focus: {uuid:waypoint_uuid, type:'waypoint'},
                    graphData: null
                }
            }
        })
    })
    // Enumerate locations and add warnings for those not used in the program.
    Object.values(program.data.locations).forEach(location=>{
        if (!location.joints.reachable && usedPoses.indexOf(location.uuid) < 0) {
            usedPoses.push(location.uuid)
            const uuid = generateUuid('issue');
            issues[uuid] = {
                uuid: uuid,
                requiresChanges: false,
                title: `Unused Location "${location.name}" not reachable`,
                description: `Location "${location.name}" is not reachable by the robot, but isn't used in the program.`,
                complete: false,
                focus: {uuid:location.uuid, type:'location'},
                graphData: null
            }
        }
    })
    // Enumerate waypoints and add warnings for those not used in the program.
    Object.values(program.data.waypoints).forEach(waypoint=>{
        if (!waypoint.joints.reachable && usedPoses.indexOf(waypoint.uuid) < 0) {
            usedPoses.push(waypoint.uuid)
            const uuid = generateUuid('issue');
            issues[uuid] = {
                uuid: uuid,
                requiresChanges: false,
                title: `Unused Waypoint "${waypoint.name}" not reachable`,
                description: `Waypoint "${waypoint.name}" is not reachable by the robot, but isn't used in the program.`,
                complete: false,
                focus: {uuid:waypoint.uuid, type:'waypoint'},
                graphData: null
            }
        }
    })
    return issues;
}

export const findJointSpeedIssues = (program) => { // requires trace processor, joint speed grader (can also use intermediate type)
    let issues = {};

    return issues;
}

export const findEndEffectorSpeedIssues = (program) => { // requires trace processor, end effector grader + intermediate end effector speed interediate type
    let issues = {};

    return issues;
}

export const findPayloadIssues = (program) => { // Shouldn't change during a trajectory so more of a check on thing weight vs. robot payload (e.g., 3kg in 1g)
    let issues = {};

    return issues;
}

export const findSpaceUsageIssues = (program) => { // Requires a convex hall operation on joint frames in traces. This volume can be compared against whole workcell (fraction) and can be used for intersection with extruded human occupancy zones 
    let issues = {};

    return issues;
}