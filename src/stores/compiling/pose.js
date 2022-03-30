import { STATUS, ROOT_BOUNDS } from "../Constants";
import { Quaternion } from 'three';
import { distance, likStateToData, createEnvironmentModel, createStaticEnvironment } from "../helpers";


export const poseCompiler = ({data, properties, objectTypes, context, path, memo, module, worldModel}) => {

    // const solver = new module.Solver(properties.urdf,[
    //     {type:'PositionMatch',name:"EE Position",link:"wrist_3_link",weight:50},
    //     {type:'OrientationMatch',name:"EE Rotation",link:"wrist_3_link",weight:25},
    //     {type:'CollisionAvoidance',name:"Collision Avoidance",weight:2}
    //   ],ROOT_BOUNDS, createStaticEnvironment({}), null, false, 1, 450);
    // const currentTime = Date.now();
    // let status = STATUS.FAILED;
    // let state = {};
    // const pos = properties.position;
    // const rot = properties.rotation;
    // const goals = [
    //     {Translation:[pos.x,pos.y,pos.z]},
    //     {Rotation:[rot.w,rot.x,rot.y,rot.z]},
    //     null
    // ]
    // while (Date.now() - currentTime < 5000 && status === STATUS.FAILED) {
    //     state = solver.solve(goals, [50, 25, 2]);
    //     const p = state.frames.wrist_3_link.translation;
    //     const r = state.frames.wrist_3_link.rotation;
    //     const achievedPos = {x:p[0],y:p[1],z:p[2]}
    //     const goalQuat = new Quaternion(rot.x,rot.y,rot.z,rot.w)
    //     const achievedQuat = new Quaternion(r[1],r[2],r[3],r[0])
    //     const translationDistance = distance(achievedPos,pos);
    //     const rotationalDistance = goalQuat.angleTo(achievedQuat);
    //     if (translationDistance < 0.01 && rotationalDistance < 0.01) {
    //         status = STATUS.VALID
    //     }
    // }

    const newCompiled = {status: STATUS.VALID};
    return newCompiled
}