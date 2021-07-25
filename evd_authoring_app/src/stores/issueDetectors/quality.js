import { generateUuid } from "../generateUuid"

export const findMissingBlockIssues = (program) => {
    let issues = {};
    // Enumerate all primitives and notify if they are missing any trajectories
    Object.values(program.data.primitives).forEach(primitive=>{
        if (primitive.type === 'node.primitive.move-trajectory.') {
            if (primitive.parameters.trajectory_uuid === null) {
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    uuid: uuid,
                    requiresChanges: true,
                    title: `Missing trajectory block`,
                    description: `A 'Move Trajectory' action is missing a required trajectory.`,
                    complete: false,
                    focus: {uuid:primitive.uuid, type:'primitive'},
                    graphData: null
                }
            }
        }
    })
    // More missing blocks could be added later
    return issues;
}

export const findMissingParameterIssues = (program) => {
    let issues = {};
    // Enumerate all primitives
    Object.values(program.data.primitives).forEach(primitive=>{
        Object.keys(primitive.parameters).forEach(parameterName=>{
            if (parameterName === 'thing_uuid' && primitive.parameters.thing_uuid === null) {
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    uuid: uuid,
                    requiresChanges: true,
                    title: `Missing Thing parameter in action`,
                    description: `This action does not have a defined Thing, and needs this value to be functional.`,
                    complete: false,
                    focus: {uuid:primitive.uuid, type:'primitive'},
                    graphData: null
                }
            }
        })
    })
    return issues;
}

export const findUnusedSkillIssues = (program) => {
    let issues = {};

    return issues;
}

export const findUnusedFeatureIssues = (program) => {
    let issues = {};

    return issues;
}

export const findEmptyBlockIssues = (program) => {
    let issues = {};

    return issues;
}