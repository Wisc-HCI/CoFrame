import { DATA_TYPES } from "simple-vp";
import { generateUuid } from "../generateUuid";
import { pickBy } from 'lodash';

export const findMissingBlockIssues = ({programData}) => {
    let issues = {};
    // Enumerate all primitives and notify if they are missing any trajectories
    Object.values(programData).filter(v => v.type === 'moveTrajectoryType').forEach(moveTrajectory => {
        if (moveTrajectory.properties.trajectory === null) {
            const uuid = generateUuid('issue');
                issues[uuid] = {
                    id: uuid,
                    requiresChanges: true,
                    title: `Missing trajectory block`,
                    description: `A 'Move Trajectory' action is missing a required trajectory.`,
                    complete: false,
                    focus: [moveTrajectory.id],
                    graphData: null
                }
        }
    })
    // More missing blocks could be added later
    return [issues, {}];
}

export const findMissingParameterIssues = ({programData, programSpec}) => {
    let issues = {};
    // Enumerate all primitives
    Object.values(programData).forEach(primitive=>{
        // Enumearate the parameter types
        if (primitive.properties) {
            Object.keys(primitive.properties).forEach(parameterName=>{
                let nullIsValid = programSpec.objectTypes[primitive.type].properties[parameterName] ? programSpec.objectTypes[primitive.type].properties[parameterName].nullValid : false;
                if (!nullIsValid && primitive.properties[parameterName] === null) {
                    if (parameterName === 'thing') {
                        const uuid = generateUuid('issue');
                        issues[uuid] = {
                            id: uuid,
                            requiresChanges: true,
                            title: `Missing Thing parameter in action`,
                            description: `This action does not have a defined Thing, and needs this value to be functional.`,
                            complete: false,
                            focus: [primitive.id],
                            graphData: null
                        }
                    }
                    if (parameterName === 'machine') {
                        const uuid = generateUuid('issue');
                        issues[uuid] = {
                            id: uuid,
                            requiresChanges: true,
                            title: `Missing Machine parameter in action`,
                            description: `This action does not have a defined Machine, and needs this value to be functional.`,
                            complete: false,
                            focus: [primitive.id],
                            graphData: null
                        }
                    }
                    if (primitive.dataType === DATA_TYPES.INSTANCE && (parameterName === 'startLocation' || parameterName === 'endLocation')) {
                        const uuid = generateUuid('issue');
                        issues[uuid] = {
                            id: uuid,
                            requiresChanges: true,
                            title: `Missing Location parameter in action`,
                            description: `This action does not have a defined Location, and needs this value to be functional.`,
                            complete: false,
                            focus: [primitive.id],
                            graphData: null
                        }
                    }
                    if (parameterName === 'trajectory') {
                        const uuid = generateUuid('issue');
                        issues[uuid] = {
                            id: uuid,
                            requiresChanges: true,
                            title: `Missing Trajectory parameter in action`,
                            description: `This action does not have a defined Trajectory, and needs this value to be functional.`,
                            complete: false,
                            focus: [primitive.id],
                            graphData: null
                        }
                    }
                    if (parameterName === 'process') {
                        const uuid = generateUuid('issue');
                        issues[uuid] = {
                            id: uuid,
                            requiresChanges: true,
                            title: `Missing Process parameter in action`,
                            description: `This action does not have a defined Process, and needs this value to be functional.`,
                            complete: false,
                            focus: [primitive.id],
                            graphData: null
                        }
                    }
                }
            })
        }
    })

    Object.values(programData).filter(v => v.type === "trajectoryType" && v.dataType === DATA_TYPES.INSTANCE).forEach(trajectory=>{
        if (!trajectory.properties.startLocation) {
            const uuid = generateUuid('issue');
                issues[uuid] = {
                    id: uuid,
                    requiresChanges: true,
                    title: `Missing Start Location in trajectory`,
                    description: `This trajectory needs a Start Location to be specified to be functional.`,
                    complete: false,
                    focus: [trajectory.id],
                    graphData: null
                }
        }
        if (!trajectory.properties.endLocation) {
            const uuid = generateUuid('issue');
                issues[uuid] = {
                    id: uuid,
                    requiresChanges: true,
                    title: `Missing End Location in trajectory`,
                    description: `This trajectory needs a End Location to be specified to be functional.`,
                    complete: false,
                    focus: [trajectory.id],
                    graphData: null
                }
        }
    })

    return [issues, {}];
}

export const findUnusedSkillIssues = ({programData}) => {
    // Right now we do a naive search of skill calls and indicate any that have no calls in the program. 
    // However, this may not catch all cases where they are not used, in cases where that skill calls occurs in an unused skill.
    // This could be improved in the future.

    let issues = {};
    const allSkills = Object.values(programData).filter(v => v.type === "skillType" && v.dataType !== DATA_TYPES.CALL).map(v => { return v.id});
    let usedSkills = [];
    Object.values(programData).filter(v => v.type === "skillType" && v.dataType=== DATA_TYPES.CALL).forEach(primitive=>{
        if (primitive.ref) {
            usedSkills.push(primitive.ref)
        }
    })

    const unusedSkills = allSkills.filter(skill=>usedSkills.indexOf(skill)<0);
    unusedSkills.forEach(skill_uuid=>{
        const skill = programData[skill_uuid];
        const uuid = generateUuid('issue');
        issues[uuid] = {
            id: uuid,
            requiresChanges: false,
            title: `Unused Skill "${skill.name}"`,
            description: `This skill is not used by the program. Consider removing it for simplicity.`,
            complete: false,
            focus: [skill_uuid],
            graphData: null
        }
    })
    return [issues, {}];
}

export const findUnusedFeatureIssues = ({programData}) => {
    // Right now we do a naive search of primitives and indicate any that have no references to features in the program. 
    // However, this may not catch all cases where they are not used, in cases where that usage occurs in an unused skill.
    // This could be improved in the future.

    let issues = {};
    const allLocations = Object.keys(pickBy(programData, function (v) { return (v.dataType === DATA_TYPES.INSTANCE || v.dataType === DATA_TYPES.ARGUMENT) && v.type === 'locationType' }));
    const allWaypoints = Object.keys(pickBy(programData, function (v) { return (v.dataType === DATA_TYPES.INSTANCE || v.dataType === DATA_TYPES.ARGUMENT) && v.type === 'waypointType'}));
    const allMachines = Object.keys(pickBy(programData, function (v) { return (v.dataType === DATA_TYPES.INSTANCE || v.dataType === DATA_TYPES.ARGUMENT) && v.type === 'machineType'}));
    const allThingPlaceholders = Object.keys(pickBy(programData, function (v) { return (v.dataType === DATA_TYPES.INSTANCE || v.dataType === DATA_TYPES.ARGUMENT) && v.type === 'thingType'}));
    const allTrajectories = Object.keys(pickBy(programData, function (v) { return (v.dataType === DATA_TYPES.INSTANCE || v.dataType === DATA_TYPES.ARGUMENT) && v.type === 'trajectoryType'}));
    const allProcesses = Object.keys(pickBy(programData, function (v) { return (v.dataType === DATA_TYPES.INSTANCE || v.dataType === DATA_TYPES.ARGUMENT) && v.type === 'processType'}));

    let usedLocations = [];
    let usedWaypoints = [];
    let usedMachines = [];
    let usedThingPlaceholders = [];
    let usedTrajectories = [];
    let usedProcesses = [];
    
    // First, enumerate primitives and check for usage
    Object.values(programData).forEach(primitive=>{
        if (primitive.dataType !== DATA_TYPES.CALL && primitive.properties) {
            // First, handle the cases where the primitives are simple
            Object.keys(primitive.properties).forEach(paramKey=>{
                if (paramKey === 'thing' && primitive.properties[paramKey] !== null) {
                    let thing = programData[primitive.properties[paramKey]].ref;
                    usedThingPlaceholders.push(thing);
                } else if (paramKey === 'machine' && primitive.properties[paramKey] !== null) {
                    let machine = programData[primitive.properties[paramKey]].ref;
                    usedMachines.push(machine)
                } else if (paramKey === 'process' && primitive.properties[paramKey] !== null) {
                    let process = programData[primitive.properties[paramKey]].ref;
                    usedProcesses.push(process)
                } else if (paramKey === 'trajectory' && primitive.properties[paramKey] !== null) {
                    let trajectory = programData[primitive.properties[paramKey]].dataType === DATA_TYPES.INSTANCE ? programData[primitive.properties[paramKey]].id : programData[primitive.properties[paramKey]].ref;
                    usedTrajectories.push(trajectory)
                } else if (paramKey === 'startLocation' && primitive.properties[paramKey] !== null) {
                    let location = programData[primitive.properties[paramKey]].ref;
                    usedLocations.push(location);
                } else if (paramKey === 'endLocation' && primitive.properties[paramKey] !== null) {
                    let location = programData[primitive.properties[paramKey]].ref;
                    usedLocations.push(location);
                } else if (paramKey === 'waypoints' && primitive.properties[paramKey] !== []) {
                    primitive.properties[paramKey].forEach(wp => {
                        let waypoint = programData[wp].ref;
                        usedWaypoints.push(waypoint);
                    });
                }
            });
        } else {
            // In cases with skill-calls, check the corresponding skills for the types and do matchmaking
            const skillInfo = programData[primitive.ref];
            if (skillInfo.arguments) {
                skillInfo.arguments.forEach(argument=>{
                    if (primitive.properties[argument] !== null && programData[primitive.properties[argument]]) {
                        if (programData[argument].type === 'machineType') {
                            usedMachines.push(programData[primitive.properties[argument]].ref)
                        } else if (programData[argument].type === 'locationType') {
                            usedLocations.push(programData[primitive.properties[argument]].ref)
                        } else if (programData[argument].type === 'waypointType') {
                            usedWaypoints.push(programData[primitive.properties[argument]].ref)
                        } else if (programData[argument].type === 'thingType') {
                            usedThingPlaceholders.push(programData[primitive.properties[argument]].ref)
                        } else if (programData[argument].type === 'trajectoryType') {
                            usedTrajectories.push(programData[primitive.properties[argument]].ref)
                        } else if (programData[argument].type === 'processType') {
                            usedProcesses.push(programData[primitive.properties[argument]].ref)
                        }
                    }
                });
            }
        }
    });

    const unusedLocations = allLocations.filter(uuid => usedLocations.indexOf(uuid)<0);
    const unusedWaypoints = allWaypoints.filter(uuid => usedWaypoints.indexOf(uuid)<0);
    const unusedMachines = allMachines.filter(uuid => usedMachines.indexOf(uuid)<0);
    const unusedThingPlaceholders = allThingPlaceholders.filter(uuid => usedThingPlaceholders.indexOf(uuid)<0);
    const unusedTrajectories = allTrajectories.filter(uuid => usedTrajectories.indexOf(uuid)<0);
    const unusedProcesses = allProcesses.filter(uuid => usedProcesses.indexOf(uuid)<0);

    unusedLocations.forEach(unused_feature_uuid=>{
        const location = programData[unused_feature_uuid];
        const uuid = generateUuid('issue');
        issues[uuid] = {
            id: uuid,
            requiresChanges: false,
            title: `Unused Location "${location.name}"`,
            description: `This location is not used by the program. Consider removing it for simplicity.`,
            complete: false,
            focus: [unused_feature_uuid],
            graphData: null
        }
    });

    unusedWaypoints.forEach(unused_feature_uuid=>{
        const waypoint = programData[unused_feature_uuid];
        const uuid = generateUuid('issue');
        issues[uuid] = {
            id: uuid,
            requiresChanges: false,
            title: `Unused Waypoint "${waypoint.name}"`,
            description: `This waypoint is not used by the program. Consider removing it for simplicity.`,
            complete: false,
            focus: [unused_feature_uuid],
            graphData: null
        }
    });

    unusedMachines.forEach(unused_feature_uuid=>{
        const machine = programData[unused_feature_uuid];
        const uuid = generateUuid('issue');
        issues[uuid] = {
            id: uuid,
            requiresChanges: false,
            title: `Unused Machine "${machine.name}"`,
            description: `This machine is not used by the program. Consider removing it for simplicity.`,
            complete: false,
            focus: [unused_feature_uuid],
            graphData: null
        }
    });

    unusedThingPlaceholders.forEach(unused_feature_uuid=>{
        const placeholder = programData[unused_feature_uuid];
        const uuid = generateUuid('issue');
        // May need to modify this issue (specifically the focus)
        issues[uuid] = {
            id: uuid,
            requiresChanges: false,
            title: `Unused Thing "${placeholder.name}"`,
            description: `This thing is not used by the program. Consider removing it for simplicity.`,
            complete: false,
            focus: [unused_feature_uuid],
            graphData: null
        }
    });

    unusedTrajectories.forEach(unused_feature_uuid=>{
        const trajectory = programData[unused_feature_uuid];
        const uuid = generateUuid('issue');
        // May need to modify this issue (specifically the focus)
        issues[uuid] = {
            id: uuid,
            requiresChanges: false,
            title: `Unused Trajectory "${trajectory.name}"`,
            description: `This trajectory is not used by the program. Consider removing it for simplicity.`,
            complete: false,
            focus: [unused_feature_uuid],
            graphData: null
        }
    });
    
    unusedProcesses.forEach(unused_feature_uuid=>{
        const process = programData[unused_feature_uuid];
        const uuid = generateUuid('issue');
        // May need to modify this issue (specifically the focus)
        issues[uuid] = {
            id: uuid,
            requiresChanges: false,
            title: `Unused Process "${process.name}"`,
            description: `This process is not used by the program. Consider removing it for simplicity.`,
            complete: false,
            focus: [unused_feature_uuid],
            graphData: null
        }
    });

    return [issues, {}];
}

export const findEmptyBlockIssues = ({programData, program}) => {
    let issues = {};
    // Enumerate skills and return warnings about ones that have a primitiveIds list of length 0
    Object.values(programData).filter(v => v.dataType !== DATA_TYPES.CALL && v.type === 'skillType').forEach(skill=>{
        if (skill.properties.children.length === 0) {
            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
                requiresChanges: false,
                title: `Empty Skill: ${skill.name}`,
                description: `This skill is empty and contains no actions.`,
                complete: false,
                focus: [skill.id],
                graphData: null
            }
        }
    })
    // Enumerate hierarchical and return warnings about ones that have a primitiveIds list of length 0
    Object.values(programData).filter(v => v.type === 'hierarchicalType').forEach(primitive => {
        if (primitive.properties.children.length === 0) {
            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
                requiresChanges: false,
                title: `Empty Action or Structure`,
                description: `This structure is empty and contains no actions. Consider removing.`,
                complete: false,
                focus: [primitive.id],
                graphData: null
            }
        }
    })
    // Enumerate the program and return a warning if primitiveIds list is of length 0
    if (program.properties.children.length === 0) {
        const uuid = generateUuid('issue');
        issues[uuid] = {
            id: uuid,
            requiresChanges: true,
            title: `Program is empty`,
            description: `The program is currently empty. Add actions to make the program perform tasks.`,
            complete: false,
            focus: [program.id],
            graphData: null
        }
    }

    return [issues, {}];
}

export const findProcessLogicIssues = ({program, programData}) => { //init , started, waiting, stopped
    let issues = {};
    let machineState = [];
    let trackedActions = [];

    program.properties.compiled["{}"].steps.forEach(step => {
        let source = programData[step.source];

        if (source.type === 'machineInitType') {
            machineState[source.properties.machine] = 'init';
        } else if (source.type === 'processStartType') {
            if (!trackedActions.includes(source.id) && !(source.properties.machine in machineState)) {
                trackedActions.push(source.id);
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    id: uuid,
                    requiresChanges: true,
                    title: `Machine needs to be initialized first`,
                    description: `Cannot run a process-start before the corresponding machine's machine-initialize`,
                    complete: false,
                    focus: [source.id],
                    graphData: null,
                    sceneData : null,
                    code : null
                }
            } else if (machineState[source.properties.machine] === 'init') {
                machineState[source.properties.machine] = 'started';
            } else if (!trackedActions.includes(source.id) && machineState[source.properties.machine] === 'started') {
                trackedActions.push(source.id);
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    id: uuid,
                    requiresChanges: true,
                    title: `Machine started more than once`,
                    description: `Cannot run a process-start on the same machine more than once`,
                    complete: false,
                    focus: [source.id],
                    graphData: null,
                    sceneData : null,
                    code : null
                }
            }
        } else if (source.type === 'processWaitType') {
            if (!trackedActions.includes(source.id) && !(source.properties.machine in machineState)) {
                trackedActions.push(source.id);
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    id: uuid,
                    requiresChanges: true,
                    title: `Machine has to be started or initialized`,
                    description: `Process-wait is running on a machine that has not been started or initialized`,
                    complete: false,
                    focus: [source.id],
                    graphData: null,
                    sceneData : null,
                    code : null
                }
            } else if (machineState[source.properties.machine] === 'started') {
                machineState[source.properties.machine] = 'waiting';
            } else if (machineState[source.properties.machine] === 'init') {
                machineState[source.properties.machine] = 'waiting';
                if (!trackedActions.includes(source.id)) {
                    trackedActions.push(source.id)
                    const uuid = generateUuid('issue');
                    issues[uuid] = {
                        id: uuid,
                        requiresChanges: false,
                        title: `Machine has only been initialized`,
                        description: `Process-wait is running on a machine that has not been started`,
                        complete: false,
                        focus: [source.id],
                        graphData: null,
                        sceneData : null,
                        code : null
                    }
                }
            }
        }
    })

    // machineState.forEach(machine => {
    //     if (machine.state !== 'stop' && (machine.state === 'started'|| machine.state === 'waiting')){
    //         const uuid = generateUuid('issue');
    //         issues[uuid] = {
    //         id: uuid,
    //         requiresChanges: true,
    //         title: `Machine never stopped`,
    //         description: `A machine needs to be ended by running a machine-stop`,
    //         complete: false,
    //         focus: [machine.primitiveUUID], // ask 
    //         graphData: null,
    //         sceneData : null,
    //         code : null
    //         }          
    //     }

    // })
    return [issues, {}];
}