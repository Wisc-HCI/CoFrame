import { DATA_TYPES } from "simple-vp";
import { generateUuid } from "../generateUuid"

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
                    if (parameterName === 'startLocation' || parameterName === 'endLocation') {
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
    const allLocations = Object.keys(_.pickBy(programData, function (v) { return (v.dataType === DATA_TYPES.INSTANCE || v.dataType === DATA_TYPES.ARGUMENT) && v.type === 'locationType' }));
    const allWaypoints = Object.keys(_.pickBy(programData, function (v) { return (v.dataType === DATA_TYPES.INSTANCE || v.dataType === DATA_TYPES.ARGUMENT) && v.type === 'waypointType'}));
    const allMachines = Object.keys(_.pickBy(programData, function (v) { return (v.dataType === DATA_TYPES.INSTANCE || v.dataType === DATA_TYPES.ARGUMENT) && v.type === 'machineType'}));
    const allThingPlaceholders = Object.keys(_.pickBy(programData, function (v) { return (v.dataType === DATA_TYPES.INSTANCE || v.dataType === DATA_TYPES.ARGUMENT) && v.type === 'thingType'}));
    const allTrajectories = Object.keys(_.pickBy(programData, function (v) { return (v.dataType === DATA_TYPES.INSTANCE || v.dataType === DATA_TYPES.ARGUMENT) && v.type === 'trajectoryType'}));
    const allProcesses = Object.keys(_.pickBy(programData, function (v) { return (v.dataType === DATA_TYPES.INSTANCE || v.dataType === DATA_TYPES.ARGUMENT) && v.type === 'processType'}));

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

export const findMachineLogicIssues = ({programData}) => { //init , started, waiting, stopped
    let issues = {};
    let machineState = [];
    let first = true;
    let gripperStart = 50;
    let gripperEnd = 0;
    let duration = 0;

    programData.forEach(primitive=>{
        if (primitive.type === 'delayType') {
            let i = machineState.length-1;
            let found = false;
            while (i > -1 && found === false){
                if (machineState[i].state === 'started'){
                    machineState[i].timer += primitive.properties.duration;
                    found = true;
                }else{
                    i -= 1;
                }
            }
        } else if (primitive.type ==='gripperType') {
            let i = machineState.length-1;
            let found = false;
            
            while (i > -1 && found === false){
                if (machineState[i].state === 'started'){
                    if (first === true){
                        //gripperStart = initialTrajectory;
                        gripperEnd = primitive.parameters.position;
                        first = false;
                        duration += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                        gripperStart = primitive.parameters.position;
                        
                    }else{
                        gripperEnd = primitive.parameters.position;
                        duration += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                        gripperStart = primitive.parameters.position;
                    }  
                    machineState[i].timer += duration;
                    found = true;
                }else{
                    i -= 1;
                }
            }

        } else if(primitive.type === 'moveTrajectoryType') {
            let i = machineState.length-1;
            let found = false;
            if (primitive.parameters.trajectory_uuid === undefined || primitive.parameters.trajectory_uuid === null){
                machineState[i].timer += 0;

            }else {
            while (i > -1 && found === false){
                if (machineState[i].state === 'started'){
                    machineState[i].timer += primitive.parameters.trajectory_uuid.trace.duration;
                    found = true;
                }else{
                    i -= 1;
                }
            }
        }

        } else if (primitive.type === 'machineInitType') {
            machineState.push({'primitiveUUID': primitive.id , 'machineID': primitive.properties.machine,'processTime':0, 'timer': 0 ,'state' : 'init'});
        } else if (primitive.type === 'processStartType') {
            let i = 0;
            let found = false;
            if (primitive.parameters.machine_uuid === null ||primitive.parameters.machine_uuid === undefined ){
                const uuid = generateUuid('issue');
                issues[uuid] = {
                uuid: uuid,
                requiresChanges: true,
                title: `Machine started without any input`,
                description: `Cannot run a machine-start without any input`,
                complete: false,
                focus: [primitive.uuid],
                graphData: null,
                sceneData : null,
                code : null
            }
            }
            while (i < machineState.length && found === false){
               
                if (machineState[i].machineID === primitive.parameters.machine_uuid.uuid && machineState[i].state === 'init'){
                    
                    machineState[i].state = 'started';
                    machineState[i].processTime = primitive.parameters.machine_uuid.process_time;
                    found = true;
                } else if (machineState[i].machineID === primitive.parameters.machine_uuid.uuid && machineState[i].state === 'started'){
                    const uuid = generateUuid('issue');
                    issues[uuid] = {
                    uuid: uuid,
                    requiresChanges: true,
                    title: `Machine started more than once`,
                    description: `Cannot run a machine-start on the same machine more than once`,
                    complete: false,
                    focus: [primitive.uuid],
                    graphData: null,
                    sceneData : null,
                    code : null
                }
                machineState[i].state = 'error'; 
                found = true;
                }
                
                else{
                    i += 1;
                }

            }
            if (found === false){
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    uuid: uuid,
                    requiresChanges: true,
                    title: `Machine needs to be initialized first`,
                    description: `Cannot run a machine-start before the corresponding machine's machine-initialize`,
                    complete: false,
                    focus: [primitive.uuid],
                    graphData: null,
                    sceneData : null,
                    code : null
                }
               // machineState[i].state = 'error'; 
            }

        } else if (primitive.type === 'processWaitType') {
            let i = 0;
            let found = false;
            while (i < machineState.length && found === false){

                if (machineState[i].machineID === primitive.parameters.machine_uuid.uuid 
                    && machineState[i].state === 'started'){
                    
                    machineState[i].state = 'waiting';
                    machineState[i].timer = machineState[i].processTime;
                    found = true;


                }else if (machineState[i].machineID === primitive.parameters.machine_uuid.uuid 
                    && machineState[i].state === 'init'){

                    machineState[i].state = 'waiting';
                    machineState[i].processTime = primitive.parameters.machine_uuid.process_time;
                    machineState[i].timer = machineState[i].processTime;

                    const uuid = generateUuid('issue');
                    issues[uuid] = {
                    uuid: uuid,
                    requiresChanges: false,
                    title: `Machine has only been initialized`,
                    description: `Machine-wait is running on a machine that has not been started`,
                    complete: false,
                    focus: [primitive.uuid],
                    graphData: null,
                    sceneData : null,
                    code : null
                    }
                    

                    found = true;
                }else{
                    i += 1;
                }
            }
            if (found === false){
                const uuid = generateUuid('issue');
                issues[uuid] = {
                uuid: uuid,
                requiresChanges: true,
                title: `Machine has to be started or initialized`,
                description: `Machine-wait is running on a machine that has not been started or initialized`,
                complete: false,
                focus: [primitive.uuid],
                graphData: null,
                sceneData : null,
                code : null
                }      
                //machineState[i].state = 'error';           
            }
            

        }
        // else if (primitive.type === 'node.primitive.machine-primitive.machine-stop.') {
        //     let i = 0;
        //     let found = false;
            
        //     while (i < machineState.length && found === false){
        //         if (machineState[i].machineID === primitive.parameters.machine_uuid.uuid 
        //             && machineState[i].state === 'started'){
                        
                    
        //             if(machineState[i].timer < machineState[i].processTime){
    
        //                 const uuid = generateUuid('issue');
        //                 issues[uuid] = {
        //                     id: uuid,
        //                     requiresChanges: true,
        //                     title: `Process stopped before the timer has completed`,
        //                     description: `Process-stop is running on a process that has not completed its timer`,
        //                     complete: false,
        //                     focus: [primitive.id],
        //                     graphData: null,
        //                     sceneData : null,
        //                     code : null
        //                 }  
        //                 found = true;   
        //                 machineState[i].state = 'error';        
        //             }else{
        //                 machineState[i].state = 'stopped';
        //                 found = true; 
        //             }
        //             found = true;
        //         }else if (machineState[i].machineID === primitive.parameters.machine_uuid.uuid 
        //             && machineState[i].state === 'waiting'){
        //                 machineState[i].state = 'stopped'; 
        //                 found = true; 

        //         }
                
        //         else {
        //             i += 1;
        //         }

        //     }
        //     if (found === false){
        //         const uuid = generateUuid('issue');
        //         issues[uuid] = {
        //             id: uuid,
        //             requiresChanges: true,
        //             title: `Machine has to be started`,
        //             description: `Machine-stop is running on a machine that has not been started`,
        //             complete: false,
        //             focus: [primitive.uuid],
        //             graphData: null,
        //             sceneData : null,
        //             code : null
        //         }    
        //         //machineState[i].state = 'error';           
        //     }
        // }
    })

    machineState.forEach(machine => {
        console.log(machine);
        if (machine.state !== 'stop' && (machine.state === 'started'|| machine.state === 'waiting')){
            const uuid = generateUuid('issue');
            issues[uuid] = {
            id: uuid,
            requiresChanges: true,
            title: `Machine never stopped`,
            description: `A machine needs to be ended by running a machine-stop`,
            complete: false,
            focus: [machine.primitiveUUID], // ask 
            graphData: null,
            sceneData : null,
            code : null
            }          
        }

    })
    return [issues, {}];
}