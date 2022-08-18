import { DATA_TYPES } from "simple-vp";
import { generateUuid } from "../generateUuid";
import { pickBy } from 'lodash';
import { ERROR, MAX_GRIPPER_DISTANCE_DIFF, MAX_GRIPPER_ROTATION_DIFF, STATUS, STEP_TYPE } from "../Constants";
import { addGraspPointToModel, addToEnvironModel, createEnvironmentModel, distance, getAllChildrenFromModel, getUserDataFromModel, queryWorldPose, updateEnvironModel } from "../../helpers/geometry";
import { Quaternion } from "three";

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

    Object.values(programData).filter(v => ['processStartType', 'processWaitType'].includes(v.type) && v.properties.status === STATUS.FAILED && v.properties.errorCode === ERROR.MISMATCHED_GIZMO).forEach(failed => {
        const uuid = generateUuid('issue');
        issues[uuid] = {
            id: uuid,
            requiresChanges: true,
            title: failed.type === 'processStartType' ? `Missing Gizmo in Process-Start` : `Missing Gizmo in Process-Wait`,
            description: failed.type === 'processStartType' ? `Missing required gizmo for process-start` : `Missing required gizmo for process-wait`,
            complete: false,
            focus: [failed.id],
            graphData: null,
            sceneData : null,
            code : null
        }
    });

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
    let initMachines = [];

    // "Initialize" tools for tracking.
    Object.values(programData).filter(v => v.type === 'toolType').forEach(tool => {
        machineState[tool.id] = 'init';
    })

    program.properties.compiled["{}"].steps.forEach(step => {
        let source = programData[step.source];

        if (step.type === STEP_TYPE.LANDMARK && source.type === 'machineInitType') {
            // TODO: need reference to top level machine
            machineState[step.data.machine] = 'init';
            if (initMachines.includes(step.data.machine)) {
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    id: uuid,
                    requiresChanges: false,
                    title: `Machine is already initialized`,
                    description: `Machine has already been initialized`,
                    complete: false,
                    focus: [source.id],
                    graphData: null,
                    sceneData : null,
                    code : null
                }
            } else {
                initMachines.push(step.data.machine);
            }
        } else if (step.type === STEP_TYPE.PROCESS_START && source.type === 'processStartType') {
            // TODO: need reference to top level machine
            if (!trackedActions.includes(source.id) && !(step.data.gizmo in machineState)) {
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
            } else if (machineState[step.data.gizmo] === 'init') {
                machineState[step.data.gizmo] = 'started';
                trackedActions.includes(source.id);
            } else if (!trackedActions.includes(source.id) && machineState[step.data.gizmo] === 'started') {
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
        } else if (step.type === STEP_TYPE.ACTION_START && source.type === 'processWaitType') {
            if (!trackedActions.includes(source.id) && !(step.data.gizmo in machineState)) {
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
            } else if (machineState[step.data.gizmo] === 'started') {
                machineState[step.data.gizmo] = 'waiting';
                trackedActions.includes(source.id);
            } else if (machineState[step.data.gizmo] === 'init') {
                machineState[step.data.gizmo] = 'waiting';
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
    
    return [issues, {}];
}

export const findThingFlowIssues = ({program, programData}) => {
    let issues = {};
    let currentGrippedThing = '';
    let currentGraspPoint = '';
    let graspWidth = -1;
    let inMoveGripper = false;
    let lastMoveGripperData = {};
    let samePositionMoveGrippers = [];
    let itemExistsError = [];

    // Tracks lists of things indexed by their spawned type (example: all "blade"s are in a list indexed by "blade")
    let trackedByType = {};

    // Array of all tracking thing ids - used to know whether a given id is a thing or not
    let thingList = [];

    // Create an updatable model of the environment usng the program data
    // This is updated from the compiled data as it's encountered
    let programModel = createEnvironmentModel(programData);

    let moveGripperOrder = [];
    let gripperId = Object.values(programData).filter(d=>d.type==='gripperType'&&d.dataType===DATA_TYPES.INSTANCE)[0].id;

    program.properties.compiled["{}"].steps.forEach(step => {
        let source = programData[step.source];


        // Thing is created/spawned
        if (step.type === STEP_TYPE.SPAWN_ITEM) {
            // Create ID for tracking, and add to array
            let id = generateUuid('thing');
            thingList.push(id);

            // Use the inputOutput (that spawned the thing) as the original position
            let ioPosition = queryWorldPose(programModel, step.data.inputOutput, '');

            // Add thing to the program model
            programModel = addToEnvironModel(programModel, 
                'world', 
                id, 
                ioPosition.position, 
                ioPosition.rotation
                );

            // Add thing grasp points to the program model
            let graspPoints = programData[step.data.thing].properties.graspPoints;
            graspPoints.forEach(graspId => {
                let gID = generateUuid('graspPoint');
                programModel = addGraspPointToModel(programModel, 
                    id, 
                    gID, 
                    programData[graspId].properties.position, 
                    programData[graspId].properties.rotation, 
                    programData[graspId].properties.gripDistance
                    );
            })

            // Add thing to the tracking
            if (!(step.data.thing in trackedByType)) {
                trackedByType[step.data.thing] = [{
                    id: id
                }];
            } else {
                trackedByType[step.data.thing].push({
                    id: id
                });
            }
        }

        // Thing is consumed/destroyed
        if (step.type === STEP_TYPE.DESTROY_ITEM) {
            // Find and remove the tracked thing
            let bucket = trackedByType[step.data.thing];
            if (bucket.length === 1) {
                delete trackedByType[step.data.thing];
            } else {
                // Remove the item that was most recently tracked
                let lst = trackedByType[step.data.thing];
                let idx = 0;
                for (let i = 0; i < lst.length; i++) {
                    if (lst[i].id === previousGraspedThingID) {
                        idx = i;
                        i = lst.length;
                    }
                }
                trackedByType[step.data.thing].splice(idx, 1);
            }
        }

        // Update all links in the model
        if (step.type === STEP_TYPE.SCENE_UPDATE) {
            Object.keys(step.data.links).forEach((link) => {
                // Update the program model for each link
                programModel = updateEnvironModel(programModel, link, step.data.links[link].position, step.data.links[link].rotation);
            });
            
            if (currentGrippedThing !== '') {
                // Get world pose of the gripper offset and update the grasped object to this pose
                let gripperOffset = queryWorldPose(programModel, gripperId+'-gripOffset', '');
                programModel = updateEnvironModel(programModel, currentGrippedThing, gripperOffset.position, gripperOffset.rotation);
            }
        }

        // Update until we get the last update of the move gripper action
        if (step.type === STEP_TYPE.SCENE_UPDATE &&
            source.type === 'moveGripperType') {
                if (!inMoveGripper) {
                    inMoveGripper = true;
                }
                lastMoveGripperData = {...step};
        }

        // Once out of the move gripper action, use the last data point to calculate everything
        if (inMoveGripper && 
            !(step.type === STEP_TYPE.SCENE_UPDATE &&
              source.type === 'moveGripperType')) {
            inMoveGripper = false;

            if (!moveGripperOrder.includes(lastMoveGripperData.source)) {
                moveGripperOrder.push(lastMoveGripperData.source);
            }
            
            let mgSource = programData[lastMoveGripperData.source];
            let thing = programData[lastMoveGripperData.data.thing.id ? lastMoveGripperData.data.thing.id : lastMoveGripperData.data.thing];

            if (thing) {
                // Update model positions of all links in the gripper
                Object.keys(lastMoveGripperData.data.links).forEach((link) => {
                    programModel = updateEnvironModel(programModel, link, lastMoveGripperData.data.links[link].position, lastMoveGripperData.data.links[link].rotation);
                });

                // Get the gripper offset position/rotation
                let gripperOffset = queryWorldPose(programModel,  gripperId+'-gripOffset', '');
                let gripperRotation = new Quaternion(gripperOffset.rotation.x, gripperOffset.rotation.y, gripperOffset.rotation.z, gripperOffset.rotation.w);

                // Gripper is closing
                if (mgSource.properties.positionStart > mgSource.properties.positionEnd) {
                    let bucket = trackedByType[thing.id];

                    // Add tool to bucket
                    if (!bucket && thing.type === 'toolType') {
                        trackedByType[thing.id] = [{
                            id: thing.id
                        }];
                        bucket = trackedByType[thing.id];
    
                        // Add tool grasp points to the program model
                        let graspPoints = thing.properties.graspPoints;
                        graspPoints.forEach(graspId => {
                            let gID = generateUuid('graspPoint');
                            programModel = addGraspPointToModel(programModel, 
                                thing.id, 
                                gID, 
                                programData[graspId].properties.position, 
                                programData[graspId].properties.rotation, 
                                programData[graspId].properties.gripDistance
                                );
                        });
                    }

                    if (bucket) {
                        for (let i = 0; i < bucket.length; i++) {
                            // Get all potential grasp locations for a given thing
                            let graspPointIDs = getAllChildrenFromModel(programModel, bucket[i].id);
                            // Search over the grasp locations and determine whether any are within some
                            // tolerance of the gripper's offset position/rotation
                            let selectedId = '';
                            let selectedGraspWidth = -1;

                            if (graspPointIDs) {
                                graspPointIDs.forEach(graspPointID => {
                                    let graspPosition = queryWorldPose(programModel, graspPointID, '');
                                    let tmpGraspWidth = getUserDataFromModel(programModel, graspPointID, 'width');
                                    let graspRotation = new Quaternion(graspPosition.rotation.x, graspPosition.rotation.y, graspPosition.rotation.z, graspPosition.rotation.w);
                                    let distTol = distance(graspPosition.position, gripperOffset.position) <= MAX_GRIPPER_DISTANCE_DIFF;
                                    let rotTol = graspRotation.angleTo(gripperRotation) <= MAX_GRIPPER_ROTATION_DIFF;

                                    if (distTol && rotTol) {
                                        selectedId = graspPointID;
                                        selectedGraspWidth = tmpGraspWidth;
                                    }
                                });

                                // Update grasped thing
                                if (selectedId !== '' &&
                                    mgSource.properties.positionEnd === selectedGraspWidth &&
                                    currentGrippedThing === '') {
                                        currentGrippedThing = bucket[i].id;
                                        currentGraspPoint = selectedId;
                                        graspWidth = selectedGraspWidth;
                                }

                                // width is not enough
                                if (selectedId !== '' && mgSource.properties.positionEnd > selectedGraspWidth) {
                                    const id = generateUuid('issue');
                                    issues[id] = {
                                        id: id,
                                        requiresChanges: false,
                                        title: 'Failed to grasp thing',
                                        description: 'The move gripper\'s end position is greater than the grasping width of the thing',
                                        complete: false,
                                        focus: [mgSource.id],
                                        graphData: null,
                                        sceneData : null,
                                        code : null
                                    }
                                }

                                // width is too much
                                if (selectedId !== '' && mgSource.properties.positionEnd < selectedGraspWidth) {
                                    const id = generateUuid('issue');
                                    issues[id] = {
                                        id: id,
                                        requiresChanges: true,
                                        title: 'Gripper width is below grasp threshold',
                                        description: 'The move gripper\'s end position is less than the grasping width of the thing',
                                        complete: false,
                                        focus: [mgSource.id],
                                        graphData: null,
                                        sceneData : null,
                                        code : null
                                    }

                                    // update grip
                                    if (currentGrippedThing === '') {
                                        currentGrippedThing = bucket[i].id;
                                        currentGraspPoint = selectedId;
                                        graspWidth = selectedGraspWidth;
                                    }
                                }

                                // gripping the wrong thing
                                if (selectedId !== '' && currentGrippedThing !== '' && bucket[i].id !== currentGrippedThing) {
                                    const id = generateUuid('issue');
                                    issues[id] = {
                                        id: id,
                                        requiresChanges: true,
                                        title: 'Grasping multiple things',
                                        description: 'Another thing is currently grasped by the robot.',
                                        complete: false,
                                        focus: [mgSource.id],
                                        graphData: null,
                                        sceneData : null,
                                        code : null
                                    }
                                }
                            }
                        }
                    } else if (!itemExistsError.includes(mgSource.id)){
                        itemExistsError.push(mgSource.id);
                        const id = generateUuid('issue');
                        issues[id] = {
                            id: id,
                            requiresChanges: true,
                            title: 'Thing has not been created',
                            description: 'Attempting to grasp thing that has not yet been spawned.',
                            complete: false,
                            focus: [mgSource.id],
                            graphData: null,
                            sceneData : null,
                            code : null
                        }
                    }
                // Gripper is opening
                } else if (mgSource.properties.positionStart < mgSource.properties.positionEnd) {
                    let bucket = trackedByType[thing.id];
                    let bucketContainsCurrentGrip = bucket ? (bucket.map(e => e.id).some(e => e === currentGrippedThing)) : false;

                    // width is still grasping
                    if (bucketContainsCurrentGrip &&
                        graspWidth !== -1 &&
                        mgSource.properties.positionEnd <= graspWidth) {
                            const id = generateUuid('issue');
                            issues[id] = {
                                id: id,
                                requiresChanges: false,
                                title: 'Failed to release thing',
                                description: 'Move gripper\'s end position is less than or equal to the grasping width of the thing',
                                complete: false,
                                focus: [mgSource.id],
                                graphData: null,
                                sceneData : null,
                                code : null
                            }
                    }

                    // releasing wrong thing
                    if (currentGrippedThing !== '' && !bucketContainsCurrentGrip) {
                        const id = generateUuid('issue');
                        issues[id] = {
                            id: id,
                            requiresChanges: false,
                            title: `Releasing incorrect thing`,
                            description: `Gripper is releasing the incorrect thing`,
                            complete: false,
                            focus: [mgSource.id],
                            graphData: null,
                            sceneData : null,
                            code : null
                        }
                    }

                    // releasing somethign before grabbing something
                    if (currentGrippedThing === '' &&
                        thing && thing.id) {
                        const id = generateUuid('issue');
                        issues[id] = {
                            id: id,
                            requiresChanges: false,
                            title: `Incorrect thing release`,
                            description: 'Robot has not previously grasped a thing',
                            complete: false,
                            focus: [mgSource.id],
                            graphData: null,
                            sceneData : null,
                            code : null
                        }
                    }

                    // release thing
                    if (bucketContainsCurrentGrip &&
                        graspWidth !== -1 &&
                        mgSource.properties.positionEnd > graspWidth) {
                            currentGrippedThing = '';
                            currentGraspPoint = '';
                            graspWidth = -1;
                    }

                // Gripper has the same start and end positions
                }
            }

            if (!samePositionMoveGrippers.includes(mgSource.id) &&
                mgSource.properties.positionStart === mgSource.properties.positionEnd) {
                    samePositionMoveGrippers.push(mgSource.id);
                    const id = generateUuid('issue');
                    issues[id] = {
                        id: id,
                        requiresChanges: false,
                        title: `Gripper position did not change`,
                        description: `Gripper start and end positions are the same.`,
                        complete: false,
                        focus: [mgSource.id],
                        graphData: null,
                        sceneData : null,
                        code : null
                    }
                }
        }
    });

    // Find inconsistencies in gripper motion
    let previousEnd = -1;
    let currentStart = -1;
    let idx = 0;
    moveGripperOrder.forEach(moveGripperId => {
        let source = programData[moveGripperId];

        if (idx === 0) {
            previousEnd = source.properties.positionEnd;
        } else {
            currentStart = source.properties.positionStart;

            if (currentStart !== previousEnd) {
                const id = generateUuid('issue');
                issues[id] = {
                    id: id,
                    requiresChanges: true,
                    title: 'Gripper position mismatch',
                    description: 'Move gripper start position does not match previous move gripper end position',
                    complete: false,
                    focus: [moveGripperId],
                    graphData: null,
                    sceneData : null,
                    code : null
                }
            }

            previousEnd = source.properties.positionEnd;
        }

        idx += 1;
    });

    return [issues, {}];
}
