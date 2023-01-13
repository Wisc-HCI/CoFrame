import * as Comlink from 'comlink';
import { ROOT_PATH, PREPROCESS_TYPES, POSTPROCESS_TYPES } from './Constants';
import { handleUpdate } from './compiling';
import { DATA_TYPES } from 'simple-vp';
import { createEnvironmentModel } from '../helpers/geometry';
// import init from '@people_and_robots/lively_tk';
import init, { Solver } from 'coframe-rust';

// console.log('lik',livelyTK)
// import livelyTk from '@people_and_robots/lively_tik'
// import init from '@people_and_robots/lively_tk';
// const likURL = new URL('@people_and_robots/lively_tk',import.meta.url)


// console.log( init )
// const loadLikModule = async () => {
//     const module = await import('@people_and_robots/lively_tk');
//     // const module = await init('/node_modules/@people_and_robots/lively_tk/lively_tk_lib_bg.wasm');
//     // const module = await init(likURL);
//     // const inited = init();
//     // console.log(module.init())
//     //return ({})
//     console.log('load',module)
//     return module;
// }

// const loadCompilerModule = async () => {
//     const module = await import('../../build/coframe');
//     return module;
// }

// export const performPoseProcess = async (data) => {
//     const { urdf, pose, scene } = data;
//     // Process the data without stalling the UI
//     const module = await loadModule();
//     const solver = new module.Solver(urdf,[
//         {type:'PositionMatch',name:"EE Position",link:"wrist_3_link",weight:50},
//         {type:'OrientationMatch',name:"EE Rotation",link:"wrist_3_link",weight:25},
//         {type:'CollisionAvoidance',name:"Collision Avoidance",weight:2}
//       ],ROOT_BOUNDS, createStaticEnvironment(scene), null, false, 1, 450);
//     const currentTime = Date.now();
//     let result = {status:'failed',state:{}};
//     const pos = pose.properties.position;
//     const rot = pose.properties.orientation;
//     const goals = [
//         {Translation:[pos.x,pos.y,pos.z]},
//         {Rotation:[rot.w,rot.x,rot.y,rot.z]},
//         null
//     ]
//     while (Date.now() - currentTime < 5000 && result.status === 'failed') {
//         result.state = solver.solve(goals, [50, 25, 2]);
//         const p = result.state.frames.wrist_3_link.translation;
//         const r = result.state.frames.wrist_3_link.rotation;
//         const achievedPos = {x:p[0],y:p[1],z:p[2]}
//         const goalQuat = new Quaternion(rot.x,rot.y,rot.z,rot.w)
//         const achievedQuat = new Quaternion(r[1],r[2],r[3],r[0])
//         const translationDistance = distance(achievedPos,pos);
//         const rotationalDistance = goalQuat.angleTo(achievedQuat);
//         if (translationDistance < 0.01 && rotationalDistance < 0.01) {
//             result.status = 'success'
//         }
//     }
//     // console.log(solver.links)
//     return result
// }

export const performCompileProcess = async (data) => {
    // console.log('performCompilerProcess--inworker')
    const { programData, compiledData, objectTypes } = data;
    // Process the data without stalling the UI
    // const module = await loadLikModule();
    
    await init();
    // console.warn(welcome('Harry'));
    const module = {Solver};
    // console.log(module)
    // console.warn('performing compile process')


    // const compModule = await loadCompilerModule();
    // console.warn(compModule.welcome("Harry"));
    // const poses = Object.values(programData).filter(d=>(d.type==='waypointType'||d.type==='locationType')&&d.dataType===DATA_TYPES.INSTANCE)
    // console.warn({input:poses,output:compModule.compile(poses)});

    let root = null;
    Object.values(programData).some(v=>{
        if (v.type==='programType') {
            root = v.id;
            return true
        } else {
            return false
        }
    })

    // TODO: define scene based on the scene item instances
    const worldModel = createEnvironmentModel(programData)

    const compileModel = compiledData;

    // First, preprocess certain types:
    let memo = {};
    let compiledMemo = {};

    Object.values(programData)
        .filter(v=>PREPROCESS_TYPES.includes(v.type) && v.dataType === DATA_TYPES.INSTANCE)
        .forEach(data=>{
            // console.log('preprocessing',data)
            const computeProps = {
                data,
                objectTypes,
                context:programData,
                path:ROOT_PATH,
                memo,
                compiledMemo,
                module,
                worldModel,
                compileModel
            }
            const {memo:newMemo, compiledMemo:newCompiledMemo} = handleUpdate(computeProps);
            // console.log(newMemo)
            memo = {...memo,...newMemo};
            compiledMemo = {...compiledMemo, ...newCompiledMemo};
    })

    // This is recursive
    const computeProps = {
        data:programData[root],
        objectTypes,
        context:programData,
        path:ROOT_PATH,
        memo,
        compiledMemo,
        module,
        worldModel,
        compileModel
    }
    const {memo:newMemo, compiledMemo:newCompiledMemo} = handleUpdate(computeProps);

    memo = {...memo,...newMemo};
    compiledMemo = {...compiledMemo, ...newCompiledMemo};

    // Snag any instances which were not directly accessible that may need to be processed.
    Object.values(programData)
        .filter(v=>POSTPROCESS_TYPES.includes(v.type) && v.dataType === DATA_TYPES.INSTANCE)
        .forEach(data=>{
            const computeProps = {
                data,
                objectTypes,
                context:programData,
                path:ROOT_PATH,
                memo,
                compiledMemo,
                module,
                worldModel,
                compileModel
            }
            const {memo:newMemo, compiledMemo:newCompiledMemo} = handleUpdate(computeProps);
            memo = {...memo,...newMemo};
            compiledMemo = {...compiledMemo, ...newCompiledMemo};
    })
    // console.log('performCompilerProcess--inworker ended')
    return {data: memo, compiledData: compiledMemo };
}

Comlink.expose({performCompileProcess,test:()=>'Hi'})
// export const performCompileProcess