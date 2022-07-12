import { STATUS } from "../Constants";
// import { likStateToData, createStaticEnvironment, queryWorldPose, quaternionLog } from "../helpers";
import { likStateToData } from "../../helpers/conversion";
import {
  createStaticEnvironment,
  queryWorldPose,
  quaternionLog,
} from "../../helpers/geometry";

export const robotAgentCompiler = ({
  data,
  properties,
  module,
  worldModel,
}) => {
  const basePose = queryWorldPose(worldModel, data.id);
  const quatLog = quaternionLog(basePose.rotation);
  const rootBounds = [
    { value: basePose.position.x, delta: 0.0 },
    { value: basePose.position.y, delta: 0.0 },
    { value: basePose.position.z, delta: 0.0 }, // Translational
    { value: quatLog[0], delta: 0.0 },
    { value: quatLog[1], delta: 0.0 },
    { value: quatLog[2], delta: 0.0 }, // Rotational
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
  // console.log(properties.urdf)
  // console.log(module.solver_new)
  const fwdsolver = new module.Solver(
    properties.urdf,
    [],
    rootBounds,
    createStaticEnvironment(worldModel),
    { origin, joints: properties.initialJointState },
    false,
    1,
    450
  );
  const newCompiled = likStateToData(
    fwdsolver.currentState,
    worldModel,
    data.id
  );
  return {
    type: data.type,
    ...newCompiled,
    ...properties,
    linkInfo: fwdsolver.links,
    jointInfo: fwdsolver.joints,
    status: STATUS.VALID,
  };
};
