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
        // Enumearate the parameter types
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
            };
            if (parameterName === 'machine_uuid' && primitive.parameters.machine_uuid === null) {
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    uuid: uuid,
                    requiresChanges: true,
                    title: `Missing Machine parameter in action`,
                    description: `This action does not have a defined Machine, and needs this value to be functional.`,
                    complete: false,
                    focus: {uuid:primitive.uuid, type:'primitive'},
                    graphData: null
                }
            };
            if (parameterName === 'location_uuid' && primitive.parameters.location_uuid === null) {
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    uuid: uuid,
                    requiresChanges: true,
                    title: `Missing Location parameter in action`,
                    description: `This action does not have a defined Location, and needs this value to be functional.`,
                    complete: false,
                    focus: {uuid:primitive.uuid, type:'primitive'},
                    graphData: null
                }
            };
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
    // Enumerate skills and return warnings about ones that have a primitiveIds list of length 0
    Object.values(program.data.skills).forEach(skill=>{
        if (skill.primitiveIds.length === 0) {
            const uuid = generateUuid('issue');
            issues[uuid] = {
                uuid: uuid,
                requiresChanges: false,
                title: `Empty Skill: ${skill.name}`,
                description: `This skill is empty and contains no actions.`,
                complete: false,
                focus: {uuid:skill.uuid, type:'skill'},
                graphData: null
            }
        }
    })
    // Enumerate hierarchical and return warnings about ones that have a primitiveIds list of length 0
    Object.values(program.data.primitives).forEach(primitive=>{
        if (primitive.type.includes('hierarchical') && primitive.primitiveIds.length === 0) {
            const uuid = generateUuid('issue');
            issues[uuid] = {
                uuid: uuid,
                requiresChanges: false,
                title: `Empty Action or Structure`,
                description: `This structure is empty and contains no actions. Consider removing.`,
                complete: false,
                focus: {uuid:primitive.uuid, type:'primitive'},
                graphData: null
            }
        }
    })
    // Enumerate the program and return a warning if primitiveIds list is of length 0
    if (program.primitiveIds.length === 0) {
        const uuid = generateUuid('issue');
        issues[uuid] = {
            uuid: uuid,
            requiresChanges: true,
            title: `Program is empty`,
            description: `The program is currently empty. Add actions to make the program perform tasks.`,
            complete: false,
            focus: {uuid:program.uuid, type:'program'},
            graphData: null
        }
    }

    return issues;
}