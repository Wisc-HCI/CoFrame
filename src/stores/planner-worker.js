import * as Comlink from 'comlink';
import { Quaternion } from 'three';
import { STATUS, STEP_CALCULATOR } from './Constants';
import { stepProcessors } from './stepProcessing';
import { DATA_TYPES } from 'simple-vp';
import { pickBy } from 'lodash';

const ROOT_BOUNDS = [
    {value:0.0,delta:0.0},{value:0.0,delta:0.0},{value:0.0,delta:0.0}, // Translational
    {value:0.0,delta:0.0},{value:0.0,delta:0.0},{value:0.0,delta:0.0}  // Rotational
]

const INITIAL_STATE = [
    { 
        origin: {translation:[0,0,0],rotation:[1,0,0,0]},
        joints: {},
        frames: {}
    }
]

const distance = (pos1, pos2) => {
 return Math.sqrt(Math.pow(pos1.x-pos2.x,2)+Math.pow(pos1.y-pos2.y,2)+Math.pow(pos1.z-pos2.z,2))
}

const createStaticEnvironment = (scene) => {

    return []
}

const loadModule = async () => {
    const module = await import('@people_and_robots/lively_tk');
    return module;
}

export const performPoseProcess = async (data) => {
    const { urdf, pose, scene } = data;
    // Process the data without stalling the UI
    const module = await loadModule();
    const solver = new module.Solver(urdf,[
        {type:'PositionMatch',name:"EE Position",link:"wrist_3_link",weight:50},
        {type:'OrientationMatch',name:"EE Rotation",link:"wrist_3_link",weight:25},
        {type:'CollisionAvoidance',name:"Collision Avoidance",weight:2}
      ],ROOT_BOUNDS, createStaticEnvironment(scene), null, false, 1, 450);
    const currentTime = Date.now();
    let result = {status:'failed',state:{}};
    const pos = pose.properties.position;
    const rot = pose.properties.orientation;
    const goals = [
        {Translation:[pos.x,pos.y,pos.z]},
        {Rotation:[rot.w,rot.x,rot.y,rot.z]},
        null
    ]
    while (Date.now() - currentTime < 5000 && result.status === 'failed') {
        result.state = solver.solve(goals, [50, 25, 2]);
        const p = result.state.frames.wrist_3_link.translation;
        const r = result.state.frames.wrist_3_link.rotation;
        const achievedPos = {x:p[0],y:p[1],z:p[2]}
        const goalQuat = new Quaternion(rot.x,rot.y,rot.z,rot.w)
        const achievedQuat = new Quaternion(r[1],r[2],r[3],r[0])
        const translationDistance = distance(achievedPos,pos);
        const rotationalDistance = goalQuat.angleTo(achievedQuat);
        if (translationDistance < 0.01 && rotationalDistance < 0.01) {
            result.status = 'success'
        }
    }
    // console.log(solver.links)
    return result
}

const performStepProcess = async (data) => {
    const { urdf, programData, objectTypes } = data;
    // Process the data without stalling the UI
    const module = await loadModule();

    const context = pickBy(programData,(entry)=>entry.dataType === DATA_TYPES.INSTANCE)

    let root = null;
    Object.values(context).some(v=>{
        if (v.type==='programType') {
            root = v.id;
            return true
        } else {
            return false
        }
    })

    // TODO: define scene based on the scene item instances
    const scene = {}

    // Create the solver
    const solver = new module.Solver(urdf,[
        {type:'PositionMatch',name:"EE Position",link:"wrist_3_link",weight:50},
        {type:'OrientationMatch',name:"EE Rotation",link:"wrist_3_link",weight:25},
        {type:'CollisionAvoidance',name:"Collision Avoidance",weight:2}
      ],ROOT_BOUNDS, createStaticEnvironment(scene), null, false, 1, 450);

    // This is recursive
    console.log({stepProcessors,computeSteps: objectTypes.programType.properties.computeSteps})
    const [steps, inventory, status, _] = stepProcessors[objectTypes.programType.properties.computeSteps.default](programData[root],objectTypes,context,solver,module,urdf)
    const newData = {...inventory,[root]:{properties:{steps,status}}};
  
    return newData;
}

Comlink.expose({performPoseProcess, performStepProcess})