import { STATUS, ROOT_BOUNDS } from "../Constants";
import { Quaternion } from 'three';
import { distance, likStateToData, createEnvironmentModel, createStaticEnvironment, queryWorldPose, quaternionLog } from "../helpers";

export const agentCompiler = ({data, properties, path, context, memo, solver, module, urdf, worldModel}) => {
    const joints = solver.joints;
    const jointObjectives = joints.map(joint=>({
        type: 'JointMatch',
        name: 'Joint Match - '+joint.name,
        joint: joint.name,
        weight: 30
    }));
    const jointObjectiveWeights = jointObjectives.map(lo=>lo.weight);
    const jointObjectiveGoals = joints.map(joint=>({
        Scalar: properties.initialJointState[joint.name]
    }));
    const basePose = queryWorldPose(worldModel,data.id);
    const quatLog = quaternionLog(basePose.rotation)
    const rootBounds = [
        {value:basePose.position.x,delta:0.0},{value:basePose.position.y,delta:0.0},{value:basePose.position.z,delta:0.0}, // Translational
        {value:quatLog[0],delta:0.0},{value:quatLog[1],delta:0.0},{value:quatLog[2],delta:0.0}  // Rotational  
    ]
    const fwdsolver = new module.Solver(urdf,[
        ...jointObjectives,
        {type:'CollisionAvoidance',name:"Collision Avoidance",weight:2}
      ],rootBounds, createStaticEnvironment(worldModel), null, false, 1, 450);
    const currentTime = Date.now();
    let status = STATUS.FAILED;
    let state = {};
    const goals = [
        ...jointObjectiveGoals,
        null
    ]
    while (Date.now() - currentTime < 5000 && status === STATUS.FAILED) {
        state = fwdsolver.solve(goals, [...jointObjectiveWeights,2]);
        const passes = true;
        joints.some(joint=>{
            if (0.01 < Math.abs(state.joints[joint.name] - properties.initialJointState[joint.name])) {
                passes = false
                return true
            } else { return false }
        })
        if (passes) {
            status = STATUS.VALID
        }
    }
    console.log({state,worldModel,frame:data.id})
    const newCompiled = likStateToData(state,worldModel,data.id);
    return { ...newCompiled, status }
}