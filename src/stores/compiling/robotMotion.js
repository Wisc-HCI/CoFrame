import {
  STATUS,
  STEP_TYPE,
  ROOT_PATH,
  ERROR
} from "../Constants";
import { likStateToData } from "../../helpers/conversion";
import {
  createStaticEnvironment,
  queryWorldPose,
  eulerFromQuaternion,
  attachmentToEEPose,
} from "../../helpers/geometry";
import { merge, isEqual } from "lodash";
import { eventsToStates, statesToSteps } from ".";

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

const updateSteps = (oldSteps, oldDuration, duration) => {
  let newSteps = [];
  oldSteps.forEach(step => {
    if (step.stepType === STEP_TYPE.ACTION_START) {
      newSteps.push({...step});
    } else if (step.stepType === STEP_TYPE.ACTION_END || step.stepType === STEP_TYPE.LANDMARK) {
      let newDelay = oldSteps.length - 3 > 0 ? oldSteps[oldSteps.length - 3].delay / oldDuration * duration : 0
      newSteps.push({...step, delay: newDelay,})
    } else {
      newSteps.push({...step, delay: step.delay / oldDuration * duration});
    }
  })
  return newSteps;
};

export const robotMotionCompiler = ({
  data,
  properties,
  path,
  memo,
  compiledMemo,
  module,
  worldModel,
  compileModel
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
      duration: 0,
      poses: [],
    };
  } else if (properties.duration < 0.01) {
    return {
      status: STATUS.FAILED,
      errorCode: ERROR.INVALID_PARAMETER,
      shouldBreak: false,
      events: [],
      steps: [],
      duration: 0,
      poses: [],
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
      duration: 0,
      poses: [],
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

  if (properties.duration !== compileModel?.[data.id]?.[path]?.duration && 
      isEqual(poses, compileModel?.[data.id]?.[path]?.poses) &&
      properties.motionType === compileModel?.[data.id]?.[path]?.motionType) {
    let eventstemp = updateSteps(compileModel?.[data.id]?.[path]?.events[0].onTrigger, compileModel?.[data.id]?.[path]?.duration, properties.duration);
    let events = [
      {
        condition: robot
          ? {
              [robot.id]: { busy: false },
            }
          : {},
        onTrigger: eventstemp,
        source: data.id,
      },
    ]
    return {
      ...compileModel?.[data.id]?.[path],
      events: events,
      steps: statesToSteps(eventsToStates(events)),
      duration: properties.duration,
      motionType: properties.motionType
    };
  } else if (properties.duration === compileModel?.[data.id]?.[path]?.duration && 
    properties.motionType === compileModel?.[data.id]?.[path]?.motionType &&
    isEqual(poses, compileModel?.[data.id]?.[path]?.poses)) {
      return {...compileModel?.[data.id]?.[path]};
    }

  robots.forEach((robot)=>{
    console.log("Trajectory planning - robot",robot.id);
    const basePose = queryWorldPose(worldModel, robot.id);
    const baseEuler = eulerFromQuaternion(
      [basePose.rotation.w, basePose.rotation.x, basePose.rotation.y, basePose.rotation.z],
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
        
        const planResult = module.planTrajectory(urdf,jointWps,staticEnvironment,rootBounds,attachmentLink,motionType==='IK');

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
    duration: duration,
    poses: poses,
    motionType: motionType,
  };

  return newCompiled;


};

