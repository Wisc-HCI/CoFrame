import { STATUS } from "../Constants";
import { likStateToData } from "../../helpers/conversion";
import { createStaticEnvironment, queryWorldPose, quaternionLog } from "../../helpers/geometry";

export const humanAgentCompiler = ({data, properties, module, worldModel}) => {
    const basePose = queryWorldPose(worldModel,data.id);
    // const quatLog = quaternionLog(basePose.rotation)
    // const rootBounds = [
    //     {value:basePose.position.x,delta:0.0},{value:basePose.position.y,delta:0.0},{value:basePose.position.z,delta:0.0}, // Translational
    //     {value:quatLog[0],delta:0.0},{value:quatLog[1],delta:0.0},{value:quatLog[2],delta:0.0}  // Rotational  
    // ]
    const origin = {
        translation:[basePose.position.x,basePose.position.y,basePose.position.z],
        rotation:[basePose.rotation.x,basePose.rotation.y,basePose.rotation.z,basePose.rotation.w]
    }
    // const fwdsolver = new module.computePose(properties.urdf,[],rootBounds, createStaticEnvironment(worldModel), {origin, joints:properties.initialJointState}, false, 1, 450);
    const newCompiled = likStateToData(module.computeForward(properties.urdf,{origin, joints:properties.initialJointState},createStaticEnvironment(worldModel)).state);
    return { ...newCompiled, status: STATUS.VALID }
}