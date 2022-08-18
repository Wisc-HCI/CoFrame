import { STATUS } from "../Constants";
// import { likStateToData, createStaticEnvironment, queryWorldPose, quaternionLog } from "../helpers";
import { likStateToData } from "../../helpers/conversion";
import {
  createStaticEnvironment,
  queryWorldPose,
  // quaternionLog,
  eulerFromQuaternion,
} from "../../helpers/geometry";

export const robotAgentCompiler = ({
  data,
  properties,
  module,
  worldModel,
}) => {
  const basePose = queryWorldPose(worldModel, data.id);
  const baseEuler = eulerFromQuaternion([basePose.w, basePose.x, basePose.y, basePose.z],'sxyz');
  // const quatLog = quaternionLog(basePose.rotation);
  const rootBounds = [
    { value: basePose.position.x, delta: 0.0 },
    { value: basePose.position.y, delta: 0.0 },
    { value: basePose.position.z, delta: 0.0 }, // Translational
    { value: baseEuler[0], delta: 0.0 },
    { value: baseEuler[1], delta: 0.0 },
    { value: baseEuler[2], delta: 0.0 }, // Rotational
  ];
  const origin = {
    translation: [
      basePose.position.x,
      basePose.position.y,
      basePose.position.z,
    ],
    rotation: [
      basePose.rotation.x,
      basePose.rotation.y,
      basePose.rotation.z,
      basePose.rotation.w,
    ],
  };
  // console.log(properties.urdf);
  // console.log(rootBounds);
  // console.log({ origin, joints: properties.initialJointState })

  // console.log(module.solver_new)
  const fwdsolver = new module.Solver(
    properties.urdf,
    [],
    rootBounds,
    createStaticEnvironment(worldModel),
    { origin, joints: properties.initialJointState },
    false,
    1,
    450,
    null
  );
  const newCompiled = likStateToData(fwdsolver.currentState,data.id,properties.linkParentMap);
  // console.log('newCompiled',newCompiled)
  return {
    type: data.type,
    ...newCompiled,
    ...properties,
    linkInfo: fwdsolver.links,
    jointInfo: fwdsolver.joints,
    status: STATUS.VALID,
  };
};