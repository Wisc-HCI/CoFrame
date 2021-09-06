import { useSceneStore } from 'robot-scene';
import lodash from 'lodash';

export const sceneSetItems = useSceneStore.getState().setItems;
export const sceneSetLines = useSceneStore.getState().setLines;
export const sceneSetTfs = useSceneStore.getState().setTfs;
export const sceneSetHulls = useSceneStore.getState().setHulls;
export const sceneSetItem = useSceneStore.getState().setItem;
export const sceneRemoveItem = useSceneStore.getState().removeItem;
export const sceneSetItemName = useSceneStore.getState().setItemShowName;
export const sceneSetItemPosition = useSceneStore.getState().setItemPosition;
export const sceneSetItemRotation = useSceneStore.getState().setItemRotation;
export const sceneSetItemScale = useSceneStore.getState().setItemScale;
export const sceneSetItemColor = useSceneStore.getState().setItemColor;
export const sceneSetItemHighlighted = useSceneStore.getState().setItemHighlighted;
export const sceneSetTF = useSceneStore.getState().setTF;
export const sceneRemoveTF = useSceneStore.getState().removeTF;
export const sceneSetTfPosition = useSceneStore.getState().setTfPosition;
export const sceneSetTfRotation = useSceneStore.getState().setTfRotation;
export const sceneSetHull = useSceneStore.getState().setHull;
export const sceneRemoveHull = useSceneStore.getState().removeHull;
export const sceneSetHullName = useSceneStore.getState().setHullName;
export const sceneSetHullVertices = useSceneStore.getState().setHullVertices;
export const sceneSetHullVertex = useSceneStore.getState().setHullVertex;
export const sceneAddHullVertex = useSceneStore.getState().addHullVertex;
export const sceneRemoveHullVertex = useSceneStore.getState().removeHullVertex;
export const sceneSetHullColor = useSceneStore.getState().setHullColor;
export const sceneSetHullHIghlighted = useSceneStore.getState().setHullHighlighted;
export const sceneSetHullOnClick = useSceneStore.getState().setHullOnClick;
export const sceneSetHullOnPointerOver = useSceneStore.getState().setHullOnPointerOver;
export const sceneSetHullOnPointerOut = useSceneStore.getState().setHullOnPointerOut;
export const sceneSetLine = useSceneStore.getState().setLine;
export const sceneRemoveLine = useSceneStore.getState().removeLine;
export const sceneSetLineName = useSceneStore.getState().setLineName;
export const sceneSetLineVertices = useSceneStore.getState().setLineVertices;
export const sceneAddLineVertex = useSceneStore.getState().addLineVertex;
export const sceneRemoveLineVertex = useSceneStore.getState().removeLineVertex;
export const sceneSetLineVertex = useSceneStore.getState().setLinevertex;

export const typeToKey = (type) => {
    let key;
    switch(type) {
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

export const HUMAN_ZONE = {radius:2,height:3,position:{x:0,y:-1,z:1}}

export const inHumanZone = ({x,y,z}) => {
    if (HUMAN_ZONE.position.z+HUMAN_ZONE.height*0.5 > z && HUMAN_ZONE.position.z-HUMAN_ZONE.height*0.5 < z) {
        return Math.pow(x - HUMAN_ZONE.position.x,2) + Math.pow(y - HUMAN_ZONE.position.y,2) < Math.pow(HUMAN_ZONE.radius,2)
    }
    return false
}

export function flattenProgram(primitives,skills,parentData) {

    let flattenedPrimitives = [];
    let flattenedSkills = [];

    primitives.forEach(primitive=>{
        if (primitive.type.includes('hierarchical')) {
            let newPrimitive = lodash.omit(primitive,'primitives');
            newPrimitive.primitiveIds = primitive.primitives.map(primitive=>primitive.uuid);
            newPrimitive.parentData = {type:'primitive',uuid:primitive}
            flattenedPrimitives.push(newPrimitive);
            const primitiveChildren = flattenProgram(primitive.primitives,[],{type:'primitive',uuid:primitive.uuid})[0];
            flattenedPrimitives = [...flattenedPrimitives, ...primitiveChildren];
        } else {
            flattenedPrimitives.push({...primitive,parentData})
        }
    });
    skills.forEach(skill=>{
        if (skill.type.includes('hierarchical')) {
            let newSkill = lodash.omit(skill,'primitives');
            newSkill.primitiveIds = skill.primitives.map(primitive=>primitive.uuid);
            flattenedSkills.push(newSkill);
            const primitiveChildren = flattenProgram(skill.primitives,[],{type:'skill',uuid:skill.uuid})[0];
            flattenedPrimitives = [...flattenedPrimitives, ...primitiveChildren]
        }
    })

    return [flattenedPrimitives,flattenedSkills]
}

export function objectMap(object, mapFn) {
    return Object.keys(object).reduce(function(result, key) {
      result[key] = mapFn(object[key],key)
      return result
    }, {})
  }

export function unFlattenProgramPrimitives(primitives, ids) {
    let unFlattenedPrimitives = [];
    console.log(primitives);
    console.log(ids);
    ids.forEach(id=>{
        let newPrimitive = lodash.omit(primitives[id],'primitiveIds');
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
    unflattenedSkillSet.forEach(skill=>{
        let newSkill = lodash.omit(skill,'primitiveIds');
        newSkill.primitives = unFlattenProgramPrimitives(primitives, skill.primitiveIds);
        unFlattenedSkills.push(newSkill);
    });
    return unFlattenedSkills;
}

export function poseToColor(pose,frame,focused) {
    let color = {r: 255, g: 255, b: 255, a: focused ? 1 : 0.25};
    if (frame === 'safety' && inHumanZone(pose.position)) {
        color.r = 255;
        color.g = 50;
        color.b = 50;
    } else if (frame === 'performance' && !pose.reachable) {
        color.r = 255;
        color.g = 50;
        color.b = 50;
    }
    return color
}

export function poseToShape(pose,frame,focused,setSecondaryFocusItem) {
    let pose_stored = pose;
    let color = poseToColor(pose_stored,frame,focused);
    const uuid = pose.uuid;
    let onClick = ()=>{};
    if (pose.type.includes('location')) {
        onClick = () => setSecondaryFocusItem('location',uuid)
    } else {
        onClick = () => setSecondaryFocusItem('waypoint',uuid)
    }

    return [
        pose.uuid,
        {
            shape: "sphere",
            name: pose.name,
            frame: "world",
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            scale: { x: 0.05, y: 0.05, z: 0.05 },
            highlighted: false,
            showName: false,
            onClick,
            position:{
                x:pose.position.x,
                y:pose.position.y,
                z:pose.position.z
            },
            color
        }
    ]
}

export function poseDataToShapes(pose,frame) {
    let pose_stored = pose;
    return [
        {
            uuid: `${pose_stored.uuid}-tag`,
            frame: 'world',
            shape: pose_stored.type.includes('location') ? 'flag' : 'tag',
            position: pose_stored.position,
            rotation: {w:1,x:0,y:0,z:0},
            scale: {x:-0.25,y:0.25,z:0.25},
            highlighted: false,
            showName: false,
            color: poseToColor(pose_stored,frame,false)
        },
        {
            uuid: `${pose_stored.uuid}-pointer`,
            frame: 'world',
            shape: pose_stored.type.includes('location') ? 'package://app/meshes/LocationMarker.stl' : 'package://app/meshes/OpenWaypointMarker.stl',
            position: pose_stored.position,
            rotation: pose_stored.orientation,
            scale: {x:1,y:1,z:1},
            highlighted: false,
            showName: false,
            color: poseToColor(pose_stored,frame,false)
        }
    ]
}

export function trajectoryDataToLine(trajectory,locations,waypoints,frame) {
    // For the time being, enumerate the location and waypoints
    let points = [];
    if (trajectory.start_location_uuid) {
        let location = locations[trajectory.start_location_uuid];
        let position = {x:location.position.x,y:location.position.y,z:location.position.z};
        points.push({position,color:poseToColor(location,frame,true)})
    }
    trajectory.waypoint_uuids.forEach(waypoint_uuid=>{
        let waypoint = waypoints[waypoint_uuid];
        let position = {x:waypoint.position.x,y:waypoint.position.y,z:waypoint.position.z};
        points.push({position,color:poseToColor(waypoint,frame,true)})
    })

    if (trajectory.end_location_uuid) {
        let location = locations[trajectory.end_location_uuid];
        let position = {x:location.position.x,y:location.position.y,z:location.position.z};
        points.push({position,color:poseToColor(location,frame,true)})
    }
    return [
        trajectory.uuid,
        {
            name: trajectory.name,
            frame: "world",
            width: 2,
            vertices: points,
            highlighted: false
        }
    ]
}

export function trajectoryDataToShapes(trajectory,locations,waypoints,frame,setSecondaryFocusItem) {
    // For the time being, enumerate the location and waypoints
    let shapes = [];
    if (trajectory.start_location_uuid) {
        let location = locations[trajectory.start_location_uuid];
        shapes.push(poseToShape(location,frame,false,setSecondaryFocusItem));
    }
    trajectory.waypoint_uuids.forEach(waypoint_uuid=>{
        let waypoint = waypoints[waypoint_uuid];
        shapes.push(poseToShape(waypoint,frame,false));
    })

    if (trajectory.end_location_uuid) {
        let location = locations[trajectory.end_location_uuid];
        shapes.push(poseToShape(location,frame,false));
    }
    return shapes
}

export const clearHighlights = () => useSceneStore.setState(state=>({
    items: objectMap(state.items,item=>({...item,highlighted:false})),
    hulls: objectMap(state.hulls,hull=>({...hull,highlighted:false}))
}))

export const highlightRobot = () => useSceneStore.setState(state=>({
    items: objectMap(state.items,(item,key)=>(key.includes('robot')?{...item,highlighted:true}:item))
}))

export const highlightGripper = () => useSceneStore.setState(state=>({
    items: objectMap(state.items,(item,key)=>(key.includes('gripper')?{...item,highlighted:true}:item))
}))

export const createTrajectory = (trajectory,locations,waypoints,frame,humanZone) => useSceneStore.setState(state=>({
    lines:{...state.lines,[trajectory.uuid]:trajectoryDataToLine(trajectory,locations,waypoints,frame,humanZone)},
    items:{...state.items,...trajectoryDataToShapes(trajectory,locations,waypoints,frame,humanZone)}
}))

export const clearTempObjects = () => useSceneStore.setState(state=>({
    // All temp objects have standard UUIDS generated by generateUuid
    items:lodash.pickBy(state.items,(_,key)=>!key.includes('-js-')),
    lines:lodash.pickBy(state.lines,(_,key)=>!key.includes('-js-')),
    hulls:lodash.pickBy(state.hulls,(_,key)=>!key.includes('-js-'))
}))

export const clearItem = (uuid) => useSceneStore.setState(state=>({
    items:lodash.omit(state.items,uuid)
}))

export const highlightSceneItem = (uuid) => useSceneStore.setState(state=>{
    let item = {...state.items[uuid],highlighted:true};
    return {items: {...state.items,[uuid]:item}}
})

export const highlightSceneHull = (uuid) => useSceneStore.setState(state=>{
    let hull = {...state.hulls[uuid],highlighted:true};
    return {hulls: {...state.hulls,[uuid]:hull}}
})