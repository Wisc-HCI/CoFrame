import { generateUuid } from "../generateUuid"

export const findMissingBlockIssues = ({program}) => {
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
    return [issues, {}];
}

export const findMissingParameterIssues = ({program}) => {
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
    return [issues, {}];
}

export const findUnusedSkillIssues = ({program}) => {
    // Right now we do a naive search of skill calls and indicate any that have no calls in the program. 
    // However, this may not catch all cases where they are not used, in cases where that skill calls occurs in an unused skill.
    // This could be improved in the future.

    let issues = {};
    const allSkills = Object.keys(program.data.skills);
    let usedSkills = [];
    Object.values(program.data.primitives).filter(primitive=>primitive.type==='node.primitive.skill-call.').forEach(primitive=>{
        if (primitive.parameters.skill_uuid) {
            usedSkills.push(primitive.parameters.skill_uuid)
        }
    })

    const unusedSkills = allSkills.filter(skill=>usedSkills.indexOf(skill)<0);
    unusedSkills.forEach(skill_uuid=>{
        const skill = program.data.skills[skill_uuid];
        const uuid = generateUuid('issue');
        issues[uuid] = {
            uuid: uuid,
            requiresChanges: false,
            title: `Unused Skill "${skill.name}"`,
            description: `This skill is not used by the program. Consider removing it for simplicity.`,
            complete: false,
            focus: {uuid:skill_uuid, type:'skill'},
            graphData: null
        }
    })
    return [issues, {}];
}

export const findUnusedFeatureIssues = ({program}) => {
    // Right now we do a naive search of primitives and indicate any that have no references to features in the program. 
    // However, this may not catch all cases where they are not used, in cases where that usage occurs in an unused skill.
    // This could be improved in the future.

    let issues = {};
    const allLocations = Object.keys(program.data.locations);
    const allWaypoints = Object.keys(program.data.waypoints);
    const allMachines = Object.keys(program.data.machines);
    const allThingPlaceholders = Object.keys(program.data.placeholders);

    let usedLocations = [];
    let usedWaypoints = [];
    let usedMachines = [];
    let usedThingPlaceholders = [];
    
    // First, enumerate primitives and check for usage
    Object.values(program.data.primitives).forEach(primitive=>{
        if (primitive.type !== 'node.primitive.skill-call.') {
            // First, handle the cases where the primitives are simple
            Object.keys(primitive.parameters).forEach(paramKey=>{
                if (paramKey === 'location_uuid' && primitive.parameters.location_uuid !== null)  {
                    usedLocations.push(primitive.parameters.location_uuid)
                } else if (paramKey === 'waypoint_uuid' && primitive.parameters.waypoint_uuid !== null)  {
                    usedWaypoints.push(primitive.parameters.waypoint_uuid)
                } else if (paramKey === 'machine_uuid' && primitive.parameters.machine_uuid !== null)  {
                    usedMachines.push(primitive.parameters.machine_uuid)
                } else if (paramKey === 'thing_uuid' && primitive.parameters.thing_uuid !== null)  {
                    usedThingPlaceholders.push(primitive.parameters.thing_uuid)
                }
            })
        } else {
            // In cases with skill-calls, check the corresponding skills for the types and do matchmaking
            const skillInfo = program.data.skills[primitive.parameters.skill_uuid];
            skillInfo.arguments.forEach(argument=>{
                const paramName = argument.parameter_key;
                if (primitive.parameters[paramName] !== null) {
                    if (argument.parameter_type === 'node.machine.') {
                        usedMachines.push(primitive.parameters[paramName])
                    } else if (argument.parameter_type === 'node.pose.waypoint.location.') {
                        usedLocations.push(primitive.parameters[paramName])
                    } else if (argument.parameter_type === 'node.pose.waypoint.') {
                        usedWaypoints.push(primitive.parameters[paramName])
                    } else if (argument.parameter_type === 'node.pose.thing.') {
                        usedThingPlaceholders.push(primitive.parameters[paramName])
                    }
                }
            })
        }
    })

    // Next, check in trajectories for locations and waypoints
    Object.values(program.data.trajectories).forEach(trajectory=>{
        if (trajectory.start_location_uuid) {
            usedLocations.push(trajectory.start_location_uuid);
        };
        if (trajectory.end_location_uuid) {
            usedLocations.push(trajectory.end_location_uuid);
        };
        trajectory.waypoint_uuids.forEach(waypoint_uuid=>usedWaypoints.push(waypoint_uuid))
    })

    const unusedLocations = allLocations.filter(uuid=>usedLocations.indexOf(uuid)<0);
    const unusedWaypoints = allWaypoints.filter(uuid=>usedWaypoints.indexOf(uuid)<0);
    const unusedMachines = allMachines.filter(uuid=>usedMachines.indexOf(uuid)<0);
    const unusedThingPlaceholders = allThingPlaceholders.filter(uuid=>usedThingPlaceholders.indexOf(uuid)<0);

    unusedLocations.forEach(unused_feature_uuid=>{
        const location = program.data.locations[unused_feature_uuid];
        const uuid = generateUuid('issue');
        issues[uuid] = {
            uuid: uuid,
            requiresChanges: false,
            title: `Unused Location "${location.name}"`,
            description: `This location is not used by the program. Consider removing it for simplicity.`,
            complete: false,
            focus: {uuid:unused_feature_uuid, type:'location'},
            graphData: null
        }
    })
    unusedWaypoints.forEach(unused_feature_uuid=>{
        const waypoint = program.data.waypoints[unused_feature_uuid];
        const uuid = generateUuid('issue');
        issues[uuid] = {
            uuid: uuid,
            requiresChanges: false,
            title: `Unused Waypoint "${waypoint.name}"`,
            description: `This waypoint is not used by the program. Consider removing it for simplicity.`,
            complete: false,
            focus: {uuid:unused_feature_uuid, type:'waypoint'},
            graphData: null
        }
    })
    unusedMachines.forEach(unused_feature_uuid=>{
        const machine = program.data.machines[unused_feature_uuid];
        const uuid = generateUuid('issue');
        issues[uuid] = {
            uuid: uuid,
            requiresChanges: false,
            title: `Unused Machine "${machine.name}"`,
            description: `This machine is not used by the program. Consider removing it for simplicity.`,
            complete: false,
            focus: {uuid:unused_feature_uuid, type:'machine'},
            graphData: null
        }
    })
    unusedThingPlaceholders.forEach(unused_feature_uuid=>{
        const placeholder = program.data.placeholders[unused_feature_uuid];
        const uuid = generateUuid('issue');
        // May need to modify this issue (specifically the focus)
        issues[uuid] = {
            uuid: uuid,
            requiresChanges: false,
            title: `Unused Thing "${placeholder.pending_node.name}"`,
            description: `This thing is not used by the program. Consider removing it for simplicity.`,
            complete: false,
            focus: {uuid:unused_feature_uuid, type:'thing'},
            graphData: null
        }
    })

    return [issues, {}];
}

export const findEmptyBlockIssues = ({program}) => {
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

    return [issues, {}];
}

export const findMachineLogicIssues = (_) => { // Hunter add code here
    let issues = {};
    // Generate issues here
    return [issues, {}];
}