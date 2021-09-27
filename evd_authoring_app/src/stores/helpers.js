import useStore from './Store';
import lodash from 'lodash';
import { GRIPPER_CONFIGURATIONS } from './gripper';
import { exportDefaultSpecifier, tsTypeAliasDeclaration } from '@babel/types';
import { UnderlineOutlined } from '@ant-design/icons';

export function durationEstimate(unrolled){
    let duration = 0;
    const initialTrajectory = 50;
    let gripperStart = 50;
    let gripperEnd = null;
    let first = true;
    let tasks = [];
    console.log("this is :" + unrolled);
    if (unrolled === undefined || unrolled === null){
        return 0;
    }else{
        Object.values(unrolled).forEach(primitive=>{
            //console.log(primitive);
            if (primitive.type === 'node.primitive.breakpoint.' ){
               return duration;

            }else if(primitive.type === 'node.primitive.delay.'){
                if (tasks.length === 0){
                    duration += primitive.parameters.duration ;
                }else{
                    let assigned = false;
                    let i = 0;
                    let waiting = false;
                    while (assigned === false && i < tasks.length){
                        if (tasks[i].status === 'started'){
                            tasks[i].timeTaken += primitive.parameters.duration ;
                            duration += primitive.parameters.duration ;
                            assigned = true;
                        }else if (tasks[i].status === 'waiting'){
                            waiting = true;
                            assigned = true;
                        }else{
                            i += 1;
                        }
                    }
                    if (i === tasks.length || waiting === false){                           
                        duration += primitive.parameters.duration ;                            
                    }
                }


            }else if (primitive.type ==='node.primitive.gripper.'){
                if (tasks.length === 0){
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
                }else{
                    let assigned = false;
                    let i = 0;
                    let waiting = false;
                    while (assigned === false && i < tasks.length){
                        if (tasks[i].status === 'started'){
                            if (first === true){
                                //gripperStart = initialTrajectory;
                                gripperEnd = primitive.parameters.position;
                                first = false;
                                duration += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                                gripperStart = primitive.parameters.position;
                                tasks[i].timeTaken += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                                
                            }else{
                                gripperEnd = primitive.parameters.position;
                                duration += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                                gripperStart = primitive.parameters.position;
                                tasks[i].timeTaken += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                            }  
                            assigned = true;
                        }else if (tasks[i].status === 'waiting'){
                            waiting = true;
                            assigned = true;
                        }else{
                            i += 1;
                        }
                    }
                    if (i === tasks.length || waiting === false){
                        gripperEnd = primitive.parameters.position;
                        duration += Math.abs((gripperEnd - gripperStart) / primitive.parameters.speed);
                        gripperStart = primitive.parameters.position;
                    }
                }           
            }else if (primitive.type === 'node.primitive.move-trajectory.'){
                if (primitive.trajectory_uuid === null){
                    duration += 0;
                }else {
                    if (tasks.length === 0){
                        duration += primitive.parameters.trajectory_uuid.trace.duration ;
                    }else{
                        let assigned = false;
                        let i = 0;
                        let waiting = true;
                        while (assigned === false && i < tasks.length){
                            if (tasks[i].status === 'started'){
                                tasks[i].timeTaken += primitive.parameters.trajectory_uuid.trace.duration ;
                                duration += primitive.parameters.trajectory_uuid.trace.duration ;
                                assigned = true;
                            }else if (tasks[i].status === 'waiting'){
                                waiting = true;
                                assigned = true;
                            }else{
                                i += 1;
                            }
                        }
                        if (i === tasks.length || waiting === false){                           
                            duration += primitive.parameters.trajectory_uuid.trace.duration ;                            
                        }
                    }
                }
                }else if (primitive.type === 'node.primitive.machine-primitive.machine-start.' ){
                tasks.push({'machineUUID' : primitive.parameters.machine_uuid.uuid, 
                           'status' : 'started', 'processTime' :primitive.parameters.machine_uuid.process_time, 'timeTaken': 0 });
                
            }else if(primitive.type === 'node.primitive.machine-primitive.machine-wait.'){
                if (tasks.length === 0){
                    console.log("this might be an error");
                }else{
                   
                    tasks.forEach((task) => 
                    {   
                        
                        if (primitive.parameters.machine_uuid.uuid === task.machineUUID &&task.status === 'started') {
                            duration +=  task.processTime - task.timeTaken;
                            console.log(duration);
                            task.timeTaken += task.processTime - task.timeTaken;
                            console.log(task.timeTaken);    
                            task.status = 'waiting';
                            
                        }
                    })
                }
            }else if (primitive.type ===  'node.primitive.machine-primitive.machine-stop.'){
                tasks.forEach((task) => {if (primitive.parameters.machine_uuid.uuid === task.machineUUID) task.status = 'stopped'} ); 
            }
        });

        tasks.forEach((task) => {if (task.status === 'closed'){

        }})
        return duration;
    }


   

}

export const typeToKey = (type) => {
    let key;
    switch (type) {
        case 'trajectory':
            key = 'trajectories';
            break;
        case 'collisionMesh':
            key = 'collisionMeshes';
            break;
        default:
            key = type + 's'
    }
    return key;
}

export const HUMAN_ZONE = { radius: 2, height: 3, position: { x: 0, y: -1, z: 1 } }

export const DEFAULT_LOCATION_COLOR = {r: 62, g: 16, b: 102, a: 1};

export const DEFAULT_WAYPOINT_COLOR = {r: 100, g: 18, b: 128, a: 1};

export const DEFAULT_TRAJECTORY_COLOR = {r: 209, g: 0, b: 146, a:1};

export const UNREACHABLE_COLOR = {r: 204, g: 75, b: 10, a: 1};

export const OCCUPANCY_ERROR_COLOR = {r: 233, g: 53, b: 152, a: 1};

export const inHumanZone = ({ x, y, z }) => {
    if (HUMAN_ZONE.position.z + HUMAN_ZONE.height * 0.5 > z && HUMAN_ZONE.position.z - HUMAN_ZONE.height * 0.5 < z) {
        return Math.pow(x - HUMAN_ZONE.position.x, 2) + Math.pow(y - HUMAN_ZONE.position.y, 2) < Math.pow(HUMAN_ZONE.radius, 2)
    }
    return false
}

export const occupancyOverlap = (position,occupancyZones) => {
    let noOverlap = true
    Object.values(occupancyZones).forEach(zone=>{
        if (position.x < zone.position_x + zone.scale_x && 
            position.x > zone.position_x - zone.scale_x && 
            position.y < zone.position_z + zone.scale_z && 
            position.y > zone.position_z - zone.scale_z ) {
            noOverlap = false
        }
    })
    return !noOverlap
}

export function flattenProgram(primitives, skills, parentData) {

    let flattenedPrimitives = [];
    let flattenedSkills = [];

    primitives.forEach(primitive => {
        if (primitive.type.includes('hierarchical')) {
            let newPrimitive = lodash.omit(primitive, 'primitives');
            newPrimitive.primitiveIds = primitive.primitives.map(primitive => primitive.uuid);
            newPrimitive.parentData = { type: 'primitive', uuid: primitive }
            flattenedPrimitives.push(newPrimitive);
            const primitiveChildren = flattenProgram(primitive.primitives, [], { type: 'primitive', uuid: primitive.uuid })[0];
            flattenedPrimitives = [...flattenedPrimitives, ...primitiveChildren];
        } else {
            flattenedPrimitives.push({ ...primitive, parentData })
        }
    });
    skills.forEach(skill => {
        if (skill.type.includes('hierarchical')) {
            let newSkill = lodash.omit(skill, 'primitives');
            newSkill.primitiveIds = skill.primitives.map(primitive => primitive.uuid);
            flattenedSkills.push(newSkill);
            const primitiveChildren = flattenProgram(skill.primitives, [], { type: 'skill', uuid: skill.uuid })[0];
            flattenedPrimitives = [...flattenedPrimitives, ...primitiveChildren]
        }
    })

    return [flattenedPrimitives, flattenedSkills]
}

function executableTrajectory(trajectory,context) {
    let executable = {uuid: trajectory.uuid, sequence: []}
    
    if (!trajectory.start_location_uuid || !trajectory.end_location_uuid) {
        console.log('no start or end')
        return null
    }
    executable.sequence.push(context[trajectory.start_location_uuid])
    trajectory.waypoint_uuids.forEach(waypoint_uuid=>{
        executable.sequence.push(context[waypoint_uuid])
    })
    executable.sequence.push(context[trajectory.end_location_uuid]);
    executable.trace = trajectory.trace;
    return executable
}

function executablePrimitiveInner(primitiveId,state,context) {
    let executable = [];
    if (primitiveId === state.uuid) {
        // Program is the primitive;
        if (state.primitiveIds.some(childId=>{
            const children = executablePrimitiveInner(childId,state,context);
            if (children === null || children.includes(null)) {
                return true
            } else {
                executable = [...executable,...children]
                return false
            }
        })) {
            // There was a null response
            return null
        } else {
            return executable
        }
    } else {
        const primitive = state.data.primitives[primitiveId];
        if (primitive.type.includes('hierarchical')) {
            if (primitive.primitiveIds.some(childId=>{
                const children = executablePrimitiveInner(childId,state,context);
                if (children === null || children.includes(null)) {
                    return true
                } else {
                    executable = [...executable,...children]
                    return false
                }
            })) {
                // There was a null response
                return null
            } else {
                return executable
            }
        } else if (primitive.type.includes('skill-call')) {
            let innerContext = {...context};
            if (Object.keys(primitive.parameters).some(parameterKey=>{
                const value = primitive.parameters[parameterKey];
                if (parameterKey !== 'skill_uuid' && context[value]) {
                    innerContext[parameterKey] = context[value]
                    return false
                } else if (parameterKey !== 'skill_uuid') {
                    return true
                }
                return false
            })) {
                // There was a null param
                return null
            }
            const calledSkill = state.data.skills[primitive.parameters.skill_uuid];
            if (calledSkill.primitiveIds.some(childId => {
                const children = executablePrimitiveInner(childId,state,innerContext);
                if (children === null || children.includes(null)) {
                    return true
                } else {
                    executable = [...executable,...children]
                    return false
                }
            })) {
                // There was a null response
                return null
            } else {
                return executable
            }
        } else if (primitive.type === 'node.primitive.breakpoint.' || primitive.type === 'node.primitive.delay.') {
            return [...executable,primitive]
        } else if (primitive.type === 'node.primitive.gripper.') {
            if (context[primitive.parameters.thing_uuid]) {
                return [...executable,{...primitive,parameters:{...primitive.parameters,thing_uuid:context[primitive.parameters.thing_uuid]}}]
            } else {
                return null
            }
        } else if (primitive.type.includes('machine-primitive')) {
            if (context[primitive.parameters.machine_uuid]) {
                return [...executable,{...primitive,parameters:{...primitive.parameters,machine_uuid:context[primitive.parameters.machine_uuid]}}]
            } else {
                return null
            }
        } else if (primitive.type === 'node.primitive.move-trajectory.') {
            if (primitive.parameters.trajectory_uuid && context[primitive.parameters.trajectory_uuid]) {
                return [...executable,{...primitive,parameters:{...primitive.parameters,trajectory_uuid:context[primitive.parameters.trajectory_uuid]}}]
            } else {
                return null
            }
        } else if (primitive.type === 'node.primitive.move-unplanned.') {
            if (context[primitive.parameters.location_uuid]) {
                return [...executable,{...primitive,parameters:{...primitive.parameters,location_uuid:context[primitive.parameters.location_uuid]}}]
            } else {
                return null
            }
        }
    }
    return null
}

export function executablePrimitive(primitiveId,state) {
    let context = {
        ...state.data.placeholders, 
        ...state.data.locations, 
        ...state.data.waypoints,
        ...state.data.machines
    }
    Object.values(state.data.trajectories).forEach(trajectory=>{
        context[trajectory.uuid] = executableTrajectory(trajectory,context)
    })
    return executablePrimitiveInner(primitiveId,state,context)
}

export function tfAnimationFromExecutable(executable,tfs) {
    let changes = objectMap(tfs,tf=>({...tf,translation:{x:[],y:[],z:[]},rotation:{w:[],x:[],y:[],z:[]}}));
    let times = [0];
    executable.forEach(chunk=>{
        if (chunk.type === 'node.primitive.gripper.') {

        } else if (chunk.type === 'node.primitive.delay.') {

        }
    })
}



function interpolateScalar(x, y) {
    //const defaultFn = (v) => 0;
    if (x.length <= 0) {
        return null
    }
    const interp = (v) => {
        const val = v > x[x.length - 1] ? v % x : v;
        let lastIdx = 0;
        for (let i = 0; i < x.length; i++) {
            if (x[i] <= v) {
                lastIdx = i
            } else {
                break
            }
        }
        return y[lastIdx]
    }
    return interp
}

export function objectMap(object, mapFn) {
    return Object.keys(object).reduce(function (result, key) {
        result[key] = mapFn(object[key], key)
        return result
    }, {})
}

export function arrayMove(arr, old_index, new_index) {
    while (old_index < 0) {
        old_index += arr.length;
    }
    while (new_index < 0) {
        new_index += arr.length;
    }
    if (new_index >= arr.length) {
        var k = new_index - arr.length + 1;
        while (k--) {
            arr.push(undefined);
        }
    }
    arr.splice(new_index, 0, arr.splice(old_index, 1)[0]);
    return arr; // for testing purposes
};

export function unFlattenProgramPrimitives(primitives, ids) {
    let unFlattenedPrimitives = [];
    ids.forEach(id => {
        let newPrimitive = lodash.omit(primitives[id], 'primitiveIds');
        if (newPrimitive.type.includes('hierarchical')) {
            newPrimitive.primitives = unFlattenProgramPrimitives(primitives, primitives[id].primitiveIds);
        };
        unFlattenedPrimitives.push(newPrimitive);
    });
    return unFlattenedPrimitives;
}

export function unFlattenProgramSkills(skills, primitives) {
    let unflattenedSkillSet = Object.values(skills);
    let unFlattenedSkills = [];
    unflattenedSkillSet.forEach(skill => {
        let newSkill = lodash.omit(skill, 'primitiveIds');
        newSkill.primitives = unFlattenProgramPrimitives(primitives, skill.primitiveIds);
        unFlattenedSkills.push(newSkill);
    });
    return unFlattenedSkills;
}

export function poseToColor(pose, frame, focused) {
    let color = { r: 255, g: 255, b: 255, a: focused ? 1 : 0 };
    if (frame === 'safety' && inHumanZone(pose.position)) {
        color.r = 233;
        color.g = 53;
        color.b = 152;
    } else if (frame === 'performance' && !pose.reachable) {
        color.r = 204;
        color.g = 75;
        color.b = 10;
    } 
    return color
}

export function reachabilityColor(focused, locationOrWaypoint){
    let color = {r: 255, g: 255, b: 255, a: focused ? 1 : 0};

        if (locationOrWaypoint === 'location') {//134, 36, 224
            color.r = 62; 
            color.g = 16; 
            color.b = 102;
        }else{//173, 31, 222
            color.r = 100;
            color.g = 18;
            color.b = 128;
        }
        return color;
    }
   

export function poseDataToShapes(pose,frame) {
    let pose_stored = pose;
    return [
        {
            uuid: `${pose_stored.uuid}-tag`,
            frame: 'world',
            name: pose.name,
            shape: pose_stored.type.includes('location') ? 'flag' : 'tag',
            position: pose_stored.position,
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            scale: { x: -0.25, y: 0.25, z: 0.25 },
            highlighted: false,
            showName: false,
            color: poseToColor(pose_stored, frame, false)
        },
        {
            uuid: `${pose_stored.uuid}-pointer`,
            frame: 'world',
            shape: pose_stored.type.includes('location') ? 'package://app/meshes/LocationMarker.stl' : 'package://app/meshes/OpenWaypointMarker.stl',
            position: pose_stored.position,
            rotation: pose_stored.orientation,
            scale: { x: 1, y: 1, z: 1 },
            highlighted: false,
            showName: false,
            color: poseToColor(pose_stored, frame, false)
        }
    ]
}

export function trajectoryDataToLine(trajectory,locations,waypoints,frame,reachableAndPerformance) {
    // For the time being, enumerate the location and waypoints //197, 50, 154
    let points = [];
    if (trajectory.start_location_uuid) {
        let location = locations[trajectory.start_location_uuid];
        let position = {x:location.position.x,y:location.position.y,z:location.position.z};
        
        if ((frame === 'performance' && location.joints.reachable)|| frame === 'quality' || frame === 'business'){
            
            points.push({position,color:{r:209,g:0,b:146,a:1}});
            
        }else{
            points.push({position,color:poseToColor(location,frame,true)});
        }

        
    }
    trajectory.waypoint_uuids.forEach(waypoint_uuid => {
        let waypoint = waypoints[waypoint_uuid];
        let position = {x:waypoint.position.x,y:waypoint.position.y,z:waypoint.position.z};
        if ((frame === 'performance' && waypoint.joints.reachable)|| frame === 'quality' || frame === 'business'){
            points.push({position,color:{r:209,g:0,b:146,a:1}})
        }else{
            points.push({position,color:poseToColor(waypoint,frame,true)}) 
        }
        
    })

    if (trajectory.end_location_uuid) {
        let location = locations[trajectory.end_location_uuid];
        let position = {x:location.position.x,y:location.position.y,z:location.position.z};
        if ((frame === 'performance' && location.joints.reachable)|| frame === 'quality' || frame === 'business'){
            points.push({position,color:{r:209,g:0,b:146,a:1}});
        }else{
            points.push({position,color:poseToColor(location,frame,true)});
        }
    }
    return {
        name: trajectory.name,
        frame: "world",
        width: 0,
        vertices: points
    }
}

export function trajectoryDataToShapes(trajectory,locations,waypoints,frame) {
    // For the time being, enumerate the location and waypoints
    let shapes = [];
    if (trajectory.start_location_uuid) {
        let location = locations[trajectory.start_location_uuid];
        shapes.push(poseDataToShapes(location, frame));
    }
    trajectory.waypoint_uuids.forEach(waypoint_uuid => {
        let waypoint = waypoints[waypoint_uuid];
        shapes.push(poseDataToShapes(waypoint, frame));
    })

    if (trajectory.end_location_uuid) {
        let location = locations[trajectory.end_location_uuid];
        shapes.push(poseDataToShapes(location, frame));
    }
    return shapes
}

export const createTrajectory = (trajectory, locations, waypoints, frame, humanZone) => useStore.setState(state => ({
    lines: { ...state.lines, [trajectory.uuid]: trajectoryDataToLine(trajectory, locations, waypoints, frame, humanZone) },
    items: { ...state.items, ...trajectoryDataToShapes(trajectory, locations, waypoints, frame, humanZone) }
}))

