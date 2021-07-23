import { generateUuid } from "../generateUuid"

export const findReachabilityIssues = (program) => {
    let issues = {};
    let usedPoses = [];
    console.log(program);
    Object.values(program.data.trajectories).forEach(trajectory=>{
        if (trajectory.start_location_uuid) {
            const startLocation = program.data.locations[trajectory.start_location_uuid];
            if (!startLocation.joints.reachable) {
                usedPoses.push(trajectory.start_location_uuid)
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    uuid: uuid,
                    requiresChanges: true,
                    description: `Location "${startLocation.name}" is not reachable by the robot, but is required for a trajectory. Please reconfigure the location to be reachable.`,
                    complete: false,
                    focus: {uuid:trajectory.start_location_uuid, type:'location'},
                    graphData: null
                }
            }
        }
        
    })
    return issues;
}

export const findJointSpeedIssues = (program) => {
    let issues = {};

    return issues;
}

export const findEndEffectorSpeedIssues = (program) => {
    let issues = {};

    return issues;
}

export const findPayloadIssues = (program) => {
    let issues = {};

    return issues;
}

export const findSpaceUsageIssues = (program) => {
    let issues = {};

    return issues;
}