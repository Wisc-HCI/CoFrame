import {
  STATUS,
  STEP_TYPE,
  ROOT_PATH,
  ERROR,
  MAX_POSE_DISTANCE_DIFF,
  MAX_POSE_ROTATION_DIFF,
  MAX_JOINT_DISTANCE_DIFF,
} from "../Constants";
// import {
//   timeGradientFunction,
//   timeGradientFunctionOneTailStart,
//   timeGradientFunctionOneTailEnd,
// } from "../helpers";
import { likStateToData } from "../../helpers/conversion";
import {
  createStaticEnvironment,
  queryWorldPose,
  // quaternionLog,
  eulerFromQuaternion,
  // distance,
  attachmentToEEPose,
} from "../../helpers/geometry";
import { merge, mapValues, fromPairs } from "lodash";
import { Quaternion, Vector3 } from "three";
import { eventsToStates, statesToSteps } from ".";

const FPS = 30;
const FRAME_TIME = FPS / 1000;
const POSITION_WEIGHT = 50;
const ROTATION_WEIGHT = 20;
const JOINT_WEIGHT = 20;
const COLLISION_WEIGHT = 5;
const SMOOTHNESS_WEIGHT = 50;
const IMPROVEMENT_THRESHOLD = 0.5;
const MS_TIME_LIMIT = 5000;
const SIMPLE_SMOOTHNESS_WEIGHT = 50;
const SIMPLE_COLLISION_WEIGHT = 5; //7;
const SIMPLE_JOINT_WEIGHT = 25;

const addOrMergeAnimation = (
  animations,
  idx,
  newLinks,
  newJoints,
  newProximity,
  reached,
  robotId,
  gripperId,
  actionId,
  attachmentLink,
  eePose,
  time
) => {
  if (animations.length > idx) {
    const currentLinks = animations[idx].data.links;
    const currentJoints = animations[idx].data.joints;
    const currentProximity = animations[idx].data.proximity;
    const currentReachability = animations[idx].data.reachability;
    const currentAttachmentPoses = animations[idx].data.attachmentPoses;
    const currentEEPoses = animations[idx].data.eePoses;
    animations[idx].data = {
      links: { ...currentLinks, ...newLinks },
      joints: { ...currentJoints, ...newJoints },
      proximity: [...currentProximity, ...newProximity],
      reachability: merge(currentReachability, {
        [robotId]: { [gripperId]: reached },
      }),
      attachmentPoses: merge(currentAttachmentPoses, {
        [robotId]: { [gripperId]: newLinks[attachmentLink] },
      }),
      eePoses: merge(currentEEPoses, {
        [robotId]: { [gripperId]: eePose },
      }),
    };
  } else {
    animations.push({
      stepType: STEP_TYPE.SCENE_UPDATE,
      data: {
        links: newLinks,
        joints: newJoints,
        proximity: newProximity,
        reachability: { [robotId]: { [gripperId]: reached } },
        attachmentPoses: {
          [robotId]: { [gripperId]: newLinks[attachmentLink] },
        },
        eePoses: {
          [robotId]: { [gripperId]: eePose },
        },
      },
      effect: {},
      source: actionId,
      delay: time, // We assume 50 frames / second
    });
  }
  return animations;
};

export const robotMotionCompiler = ({
  data,
  properties,
  path,
  memo,
  compiledMemo,
  module,
  worldModel,
}) => {

  const trajectory = properties.trajectory;
  let status = STATUS.VALID;
  let errorCode = null;
  let innerSteps = [];

  if (!trajectory.id) {
    return {
      status: STATUS.FAILED,
      errorCode: ERROR.MISSING_PARAMETER,
      shouldBreak: false,
      events: [],
      steps: [],
    };
  } else if (properties.duration < 0.01) {
    return {
      status: STATUS.FAILED,
      errorCode: ERROR.INVALID_PARAMETER,
      shouldBreak: false,
      events: [],
      steps: [],
    };
  } else if (
    trajectory.properties &&
    trajectory.properties.status === STATUS.FAILED
  ) {
    return {
      status: STATUS.FAILED,
      errorCode: ERROR.CHILD_FAILED,
      shouldBreak: false,
      events: [],
      steps: [],
    };
  }

  const staticEnvironment = createStaticEnvironment(worldModel);

  // // Enumerate the robotAgentTypes/gripperTypes currently in the memo. This is technically unsafe,
  // // but we pre-process them beforehand so it is fine. We also always assume root execution
  // // (which is fine for robots/humans/grippers).

  const grippers = Object.values(memo).filter((v) => v.type === "gripperType");
  const robots = Object.values(memo).filter((v) => v.type === "robotAgentType");
  const robot = robots[0];
  const motionType = properties.motionType;
  const duration = properties.duration;

  const poses = [
    compiledMemo[compiledMemo[trajectory.id][path].startLocation.id][path],
    ...compiledMemo[trajectory.id][path].waypoints.map(
      (wp) => compiledMemo[wp.id][path]
    ),
    compiledMemo[compiledMemo[trajectory.id][path].endLocation.id][path],
  ];

  robots.forEach((robot)=>{
    console.log("Trajectory planning - robot",robot.id);
    const basePose = queryWorldPose(worldModel, robot.id);
    const baseEuler = eulerFromQuaternion(
      [basePose.w, basePose.x, basePose.y, basePose.z],
      "sxyz"
    );
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

    const urdf = compiledMemo[robot.id][ROOT_PATH].urdf;
    const robotLinks = compiledMemo[robot.id][ROOT_PATH].linkInfo.map(
      (link) => link.name
    );
    grippers
      .filter((gripper) => robotLinks.includes(gripper.properties.relativeTo))
      .forEach(gripper=>{
        // console.log("Trajectory planning - gripper",gripper.id)
        const attachmentLink = gripper.properties.relativeTo;
        let reachableChildren = true;
        // Confirm that the poses are all valid
        poses.forEach((pose) => {
          if (!pose.reachability[robot.id][gripper.id]) {
            reachableChildren = false;
          }
        });
        if (!reachableChildren && status !== STATUS.FAILED) {
          status = STATUS.WARN;
          errorCode = ERROR.UNREACHABLE_POSE;
        } 
        const jointWps = poses.map(pose=>pose.states[robot.id][gripper.id].joints);
        
        const planResult = module.planTrajectory(urdf,jointWps,staticEnvironment,rootBounds,motionType==='IK');

        if (planResult.code === 'Failure') {
          status = STATUS.WARN;
          errorCode = ERROR.TRAJECTORY_PROGRESS;
        }
        planResult.trajectory.forEach((trajectoryState,idx)=>{
          // console.log(trajectoryState.proximity);
          const stateData = likStateToData(
            trajectoryState,
            robot.id,
            compiledMemo[robot.id][ROOT_PATH].linkParentMap
          );
          const eePose = attachmentToEEPose(
            worldModel,
            gripper.id,
            attachmentLink,
            stateData.links[attachmentLink]
          );

          innerSteps = addOrMergeAnimation(
            innerSteps,
            idx,
            stateData.links,
            stateData.joints,
            stateData.proximity,
            true,
            robot.id,
            gripper.id,
            data.id,
            attachmentLink,
            eePose,
            duration * 1000 * (idx/planResult.trajectory.length)
          );

        })
        
    })

  })

  const initialStep = {
    stepType: STEP_TYPE.ACTION_START,
    effect: robot ? { [robot.id]: { busy: true } } : {},
    data: { agent: "robot", id: data.id },
    source: data.id,
    delay: 0,
  };
  const finalStep = {
    stepType: STEP_TYPE.ACTION_END,
    effect: robot ? { [robot.id]: { busy: false } } : {},
    data: { agent: "robot", id: data.id },
    source: data.id,
    delay: innerSteps.length > 0 ? innerSteps[innerSteps.length - 1].delay : 0,
  };

  const steps =
    errorCode === ERROR.TRAJECTORY_PROGRESS
      ? [
          initialStep,
          ...innerSteps,
          {
            stepType: STEP_TYPE.LANDMARK,
            data: { label: "Robot Could Not Continue" },
            effect: {},
            source: data.id,
            delay:
              innerSteps.length > 0
                ? innerSteps[innerSteps.length - 1].delay
                : 0,
          },
          finalStep,
        ]
      : [initialStep, ...innerSteps, finalStep];

  // console.log({ errorCode, status, steps });

  const events = [
    {
      condition: robot
        ? {
            [robot.id]: { busy: false },
          }
        : {},
      onTrigger: steps,
      source: data.id,
    },
  ];

  const newCompiled = {
    status,
    errorCode,
    shouldBreak: false,
    events,
    steps: statesToSteps(eventsToStates(events)),
  };

  return newCompiled;


};

