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

const FRAME_TIME = 30;
const POSITION_WEIGHT = 30;
const ROTATION_WEIGHT = 1;
const JOINT_WEIGHT = 20;
const COLLISION_WEIGHT = 5;
const SMOOTHNESS_WEIGHT = 50;
const IMPROVEMENT_THRESHOLD = 0.5;
const MS_TIME_LIMIT = 7000;
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
  eePose
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
      delay: idx * FRAME_TIME, // We assume 50 frames / second
    });
  }
  return animations;
};

const stepIK = ({
  currentPose,
  currentJoints,
  goalPose,
  goalJoints,
  solver,
  speed,
  joints,
  attachmentLink,
  poseType,
}) => {
  const currentPosition = new Vector3(
    currentPose.position.x,
    currentPose.position.y,
    currentPose.position.z
  );

  const currentQuaternion = new Quaternion(
    currentPose.rotation.x,
    currentPose.rotation.y,
    currentPose.rotation.z,
    currentPose.rotation.w
  );

  const goalPosition = new Vector3(
    goalPose.position.x,
    goalPose.position.y,
    goalPose.position.z
  );

  const goalQuaternion = new Quaternion(
    goalPose.rotation.x,
    goalPose.rotation.y,
    goalPose.rotation.z,
    goalPose.rotation.w
  );

  const interpPos = new Vector3(0, 0, 0);
  const interpQuat = new Quaternion(0, 0, 0, 1);

  const posDist = currentPosition.distanceTo(goalPosition);
  const rotDist = currentQuaternion.angleTo(goalQuaternion);

  const jointDists = mapValues(currentJoints, (v, key) =>
    Math.abs(v - goalJoints[key])
  );

  // console.log({ posDist, rotDist, jointDists });

  let passed =
    poseType === "waypoint"
      ? posDist <= MAX_POSE_DISTANCE_DIFF * 2
      : !Object.values(jointDists).some((v) => v >= MAX_JOINT_DISTANCE_DIFF);

  if (passed) {
    console.warn(`${poseType} reached`);
    return { newState: solver.currentState, improved: true, reached: true };
  }

  if (posDist < speed / 20) {
    interpPos.copy(goalPosition);
  } else {
    interpPos
      .copy(goalPosition)
      .sub(currentPosition)
      .clampLength(0, speed / 15)
      .add(currentPosition);
  }
  if (rotDist < speed / 20) {
    interpQuat.copy(goalQuaternion);
  } else {
    interpQuat
      .copy(currentQuaternion)
      .rotateTowards(goalQuaternion, speed / 15);
  }

  const nextJoints = mapValues(currentJoints, (currentJointValue, jointKey) => {
    const signedDist = goalJoints[jointKey] - currentJointValue;
    const direction = signedDist / Math.abs(signedDist);
    const travel = (direction * speed) / 5;
    if (Math.abs(signedDist) < Math.abs(travel)) {
      return goalJoints[jointKey];
    } else {
      // console.log(travel)
      return currentJointValue + travel;
    }
  });

  let goals = {
    eePosition: { Translation: [interpPos.x, interpPos.y, interpPos.z] },
    eeRotation: { Rotation: [interpQuat.x, interpQuat.y, interpQuat.z, interpQuat.w] }
  };
  joints.forEach((j) => {
    goals[j] = { Scalar: nextJoints[j] };
  });

  let weights = {
    smoothness: SMOOTHNESS_WEIGHT,
    collision: COLLISION_WEIGHT,
    eePosition: POSITION_WEIGHT,
    eeRotation:ROTATION_WEIGHT/2
  };
  joints.forEach((j) => {
    weights[j] = poseType === "waypoint" ? JOINT_WEIGHT / 10 : JOINT_WEIGHT/2;
  });

  const newState = solver.solve(goals, weights);

  const hasMovement = joints.some((jointKey) => {
    const delta = Math.abs(newState.joints[jointKey] - currentJoints[jointKey]);
    if (delta > 0.01) {
      return true;
    }
    return false;
  });

  return { newState, improved: hasMovement, reached: false };
};

const stepJoint = ({
  currentJoints,
  goalJoints,
  solver,
  speed,
  joints,
  poseType,
}) => {
  const jointDists = mapValues(currentJoints, (v, key) =>
    Math.abs(v - goalJoints[key])
  );
  // console.log({currentJoints,goalJoints,jointDists})
  let passed = poseType === "waypoint" ?
    !Object.values(jointDists).some((v) => v >= MAX_JOINT_DISTANCE_DIFF * 3) :
    !Object.values(jointDists).some((v) => v >= MAX_JOINT_DISTANCE_DIFF);
  if (passed) {
    return { newState: solver.currentState, improved: true, reached: true };
  }
  if (passed) {
    return { newState: solver.currentState, improved: true, reached: true };
  }

  const nextJoints = mapValues(currentJoints, (currentJointValue, jointKey) => {
    const signedDist = goalJoints[jointKey] - currentJointValue;
    const direction = signedDist / Math.abs(signedDist);
    const travel = (direction * speed) / 5;
    if (Math.abs(signedDist) < Math.abs(travel)) {
      return goalJoints[jointKey];
    } else {
      // console.log(travel)
      return currentJointValue + travel;
    }
  });

  // console.log('nextJoints',nextJoints)

  let goals = {};
  joints.forEach((j) => {
    goals[j] = { Scalar: nextJoints[j] };
  });

  let weights = { smoothness: SMOOTHNESS_WEIGHT, collision: COLLISION_WEIGHT };
  joints.forEach((j) => {
    weights[j] = 20;
  });

  const newState = solver.solve(goals, weights);

  const newError = joints
    .map((jointKey) =>
      Math.abs(newState.joints[jointKey] - goalJoints[jointKey])
    )
    .reduce((next, current) => next + current);
  const oldError = joints
    .map((jointKey) => jointDists[jointKey])
    .reduce((next, current) => next + current);

  // console.log({ newError, oldError });

  const improved = newError < oldError;

  return { newState, improved, reached: false };
};

export const robotMotionCompiler = ({
  data,
  properties,
  // context,
  path,
  memo,
  // solver,
  module,
  worldModel,
  // urdf,
}) => {
  const trajectory = properties.trajectory;
  /* 
    trajectory: {
      name: "Trajectory",
      accepts: ["trajectoryType"],
      default: null,
      isList: false
    },
    velocity: {
      name: 'Velocity',
      type: SIMPLE_PROPERTY_TYPES.NUMBER,
      default: 1,
      min: 0.01,
      max: 5
    },
    motionType: {
      name: 'Motion Type',
      type: SIMPLE_PROPERTY_TYPES.OPTIONS,
      options: ['IK', 'Joint'],
      default: 'IK'
    },
    */

  let status = STATUS.VALID;
  let errorCode = null;

  if (!trajectory.id) {
    return {
      status: STATUS.FAILED,
      errorCode: ERROR.MISSING_PARAMETER,
      shouldBreak: false,
      events: [],
      steps: [],
    };
  } else if (properties.velocity < 0.01) {
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
  console.log("static env", staticEnvironment);

  // const delta = properties.positionEnd - properties.positionStart;
  // if (properties.speed === 0) {
  //     status = STATUS.FAILED;
  //     return {
  //         status,
  //         shouldBreak: false,
  //         steps: []
  //     }
  // }
  // const duration = 1000 * Math.abs(delta) / properties.speed;
  // const changePerTime = delta / duration;

  // // Enumerate the robotAgentTypes/gripperTypes currently in the memo. This is technically unsafe,
  // // but we pre-process them beforehand so it is fine. We also always assume root execution
  // // (which is fine for robots/humans/grippers).

  const grippers = Object.values(memo).filter((v) => v.type === "gripperType");
  const robots = Object.values(memo).filter((v) => v.type === "robotAgentType");
  const robot = robots[0];
  const motionType = properties.motionType;
  const velocity = properties.velocity;

  let innerSteps = [];

  // console.log(robots)

  const poses = [
    {
      type: "location",
      ...trajectory.properties.compiled[path].startLocation.properties.compiled[
        path
      ],
    },
    ...trajectory.properties.compiled[path].waypoints.map((wp) => ({
      type: "waypoint",
      ...wp.properties.compiled[path],
    })),
    {
      type: "location",
      ...trajectory.properties.compiled[path].endLocation.properties.compiled[
        path
      ],
    },
  ];

  // console.log(poses);

  robots.forEach((robot) => {
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

    // Get a list of links in the robot;
    const robotLinks = robot.properties.compiled[ROOT_PATH].linkInfo.map(
      (link) => link.name
    );
    // Get the urdf of the robot (for livelyTK)
    const urdf = robot.properties.compiled[ROOT_PATH].urdf;
    const proximity = robot.properties.compiled[ROOT_PATH].proximity;

    // Consider each gripper/robot combo.
    // We filter those combos by the ones that actually feature linkages
    grippers
      .filter((gripper) => robotLinks.includes(gripper.properties.relativeTo))
      .forEach((gripper) => {
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

        let firstLinks = poses[0].states[robot.id][gripper.id].links;
        let firstJoints = poses[0].states[robot.id][gripper.id].joints;
        // console.log('firstJoints',firstJoints);
        let firstProximity = poses[0].states[robot.id][gripper.id].proximity;
        const attachmentLink = gripper.properties.relativeTo;
        const jointNames = Object.keys(firstJoints);

        const eePose = attachmentToEEPose(
          worldModel,
          gripper.id,
          attachmentLink,
          firstLinks[attachmentLink]
        );

        // load the first frame
        innerSteps = addOrMergeAnimation(
          innerSteps,
          0,
          firstLinks,
          firstJoints,
          firstProximity,
          poses[0].reachability[robot.id][gripper.id],
          robot.id,
          gripper.id,
          data.id,
          attachmentLink,
          eePose
        );

        let improved = true;

        // Instantiate the initial state and solver;
        const standardObjectives = {
          smoothness: {
            type: "SmoothnessMacro",
            name: "Smoothness",
            weight: 40,
          },
          collision: {
            type: "CollisionAvoidance",
            name: "Collision Avoidance",
            weight: 5,
          },
        };
        const objectives =
          motionType === "IK"
            ? {
                ...standardObjectives,
                eePosition: {
                  type: "PositionMatch",
                  name: "EE Position",
                  link: attachmentLink,
                  weight: POSITION_WEIGHT,
                },
                eeRotation: {
                  type: "OrientationMatch",
                  name: "EE Rotation",
                  link: attachmentLink,
                  weight: ROTATION_WEIGHT,
                },
                ...fromPairs(jointNames.map((jointKey) => ([jointKey,{
                  type: "JointMatch",
                  name: `JointControl:${jointKey}`,
                  joint: jointKey,
                  weight: JOINT_WEIGHT / 2,
                }]))),
              }
            : {
                ...standardObjectives,
                ...fromPairs(
                  jointNames.map((jointKey) => [
                    jointKey,
                    {
                      type: "JointMatch",
                      name: `JointControl:${jointKey}`,
                      joint: jointKey,
                      weight: JOINT_WEIGHT,
                    },
                  ])
                ),
              };

        // Find the position we need in the attachment link to match the desired pose gripper position

        // console.log('goal:',goalPose)

        // Construct the joint state information
        // console.log({ objectives, motionType });
        // let state = {};

        // Construct the solver
        const initialState = { origin, joints: firstJoints, proximity };
        // console.log('initial state', initialState)
        const solver = new module.Solver(
          urdf,
          objectives,
          rootBounds,
          staticEnvironment,
          initialState,
          false,
          1,
          250,
          null
        );
        // console.log('constructed trajectory solver')
        // console.log(solver.currentState);
        // Enumerate pairs of poses.
        let poseStack = [...poses];
        let history = [
          true,
          true,
          true,
          true,
          true,
          true,
          true,
          true,
          true,
          true,
          true,
          true,
          true,
          true,
          true,
          true,
          true,
          true,
          true,
          true,
        ];

        let currentState = solver.currentState;

        let reached = false;

        let mainIdx = 0;

        // Remove the first element, since that is our initial state;
        poseStack.shift();

        let startTime = Date.now();

        while (poseStack.length > 0 && improved) {
          if (Date.now() - startTime > MS_TIME_LIMIT) {
            improved = false;
            status = STATUS.WARN;
            errorCode = ERROR.TIMEOUT;
            console.warn("timeout on trajectory calculation, cancelling");
            break;
          }

          reached = false;

          const pose = poseStack[0];
          // console.log('pose', pose);
          // const goalPose1 = pose1.goalPose;
          const goalPose = pose.goalPose;
          // const jointState1 = pose1.states[robot.id][gripper.id].joints;
          const goalJoints = pose.states[robot.id][gripper.id].joints;

          while (improved && !reached) {
            if (history.length > 30) {
              history.shift();
              // console.log("history trimmed", history);
            }

            const currentPoseRaw = currentState.frames[attachmentLink].world;
            const currentPose = {
              position: {
                x: currentPoseRaw.translation[0],
                y: currentPoseRaw.translation[1],
                z: currentPoseRaw.translation[2],
              },
              rotation: {
                x: currentPoseRaw.rotation[0],
                y: currentPoseRaw.rotation[1],
                z: currentPoseRaw.rotation[2],
                w: currentPoseRaw.rotation[3],
              },
            };

            const props = {
              currentJoints: currentState.joints,
              goalJoints,
              currentPose,
              goalPose,
              solver,
              joints: jointNames,
              speed: velocity,
              attachmentLink,
              poseType: pose.type,
            };

            const results =
              motionType === "IK" ? stepIK(props) : stepJoint(props);

            // console.log("results", results);

            const stateData = likStateToData(
              results.newState,
              robot.id,
              robot.properties.compiled[ROOT_PATH].linkParentMap
            );

            reached = results.reached;
            currentState = results.newState;
            history.push(results.improved);

            // console.log('collision between wrist_1_link and conveyorCollisionShapeBelt',currentState.proximity[1].distance)

            // Check the history, and if less than 50% of the past 30ish solutions haven't improved, kill it.
            const improvementScore =
              history.reduce(
                (previousValue, currentValue) => previousValue + currentValue
              ) / history.length;
            // console.log("improvement score:", { improvementScore, history });
            if (improvementScore < IMPROVEMENT_THRESHOLD) {
              improved = false;
              status = STATUS.WARN;
              errorCode = ERROR.TRAJECTORY_PROGRESS;
              console.warn("No longer improving, cancelling", currentState);
            }

            // console.log(stateData.links[attachmentLink])
            // stateData.frames[attachmentLink]
            const eePose = attachmentToEEPose(
              worldModel,
              gripper.id,
              attachmentLink,
              stateData.links[attachmentLink]
            );

            innerSteps = addOrMergeAnimation(
              innerSteps,
              mainIdx,
              stateData.links,
              stateData.joints,
              stateData.proximity,
              reached,
              robot.id,
              gripper.id,
              data.id,
              attachmentLink,
              eePose
            );

            mainIdx += 1;
          }
          poseStack.shift();
          // console.warn('Segment completed',{poseStack,reached,currentState})
        }
      });
  });

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

  const steps = errorCode === ERROR.TRAJECTORY_PROGRESS ? 
  [initialStep, ...innerSteps, {
    stepType: STEP_TYPE.LANDMARK,
    data: {label: 'Robot Could Not Continue'},
    effect: {},
    source: data.id,
    delay: innerSteps.length > 0 ? innerSteps[innerSteps.length - 1].delay : 0,
  }, finalStep] : 
  [initialStep, ...innerSteps, finalStep];
  

  console.log({errorCode,status,steps})

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

// const createIKGoals = (
//   goalPose1,
//   goalPose2,
//   jointState1,
//   jointState2,
//   duration,
//   jointNames,
//   type1,
//   type2
// ) => {
//   const numGoals = Math.ceil(duration / FRAME_TIME);
//   let goals = [];
//   let idx = 0;
//   const goalPos1 = new Vector3(
//     goalPose1.position.x,
//     goalPose1.position.y,
//     goalPose1.position.z
//   );
//   const goalPos2 = new Vector3(
//     goalPose2.position.x,
//     goalPose2.position.y,
//     goalPose2.position.z
//   );
//   const goalQuat1 = new Quaternion(
//     goalPose1.rotation.x,
//     goalPose1.rotation.y,
//     goalPose1.rotation.z,
//     goalPose1.rotation.w
//   );
//   const goalQuat2 = new Quaternion(
//     goalPose2.rotation.x,
//     goalPose2.rotation.y,
//     goalPose2.rotation.z,
//     goalPose2.rotation.w
//   );
//   const interpPos = new Vector3(0, 0, 0);
//   const interpQuat = new Quaternion(0, 0, 0, 1);
//   let jointGoals = { ...jointState1 };
//   while (idx <= numGoals) {
//     const percent = idx / numGoals;
//     interpPos.lerpVectors(goalPos1, goalPos2, percent);
//     interpQuat.slerpQuaternions(goalQuat1, goalQuat2, percent);
//     jointNames.forEach((jointKey) => {
//       jointGoals[jointKey] =
//         percent * jointState2[jointKey] + (1 - percent) * jointState1[jointKey];
//     });
//     const startJointGoalWeight = timeGradientFunctionOneTailStart(
//       idx,
//       10,
//       -20,
//       Math.min(numGoals / 5, 30)
//     );
//     const endJointGoalWeight =
//       type2 === "waypoint"
//         ? 0.1
//         : timeGradientFunctionOneTailEnd(
//             idx,
//             10,
//             -20,
//             Math.min(numGoals / 5, 30),
//             numGoals
//           );
//     goals.push({
//       values: [
//         null,
//         null,
//         { Translation: [interpPos.x, interpPos.y, interpPos.z] },
//         { Rotation: [interpQuat.x, interpQuat.y, interpQuat.z, interpQuat.w] },
//         // ...jointNames.map((joint) => ({ Scalar: jointState1[joint] })),
//         ...jointNames.map((joint) => ({ Scalar: jointState2[joint] })),
//       ],
//       weights: [
//         SMOOTHNESS_WEIGHT,
//         COLLISION_WEIGHT,
//         50,
//         25,
//         // ...jointNames.map(() => startJointGoalWeight),
//         ...jointNames.map(() => endJointGoalWeight),
//       ],
//       joints: jointGoals,
//       position: { x: interpPos.x, y: interpPos.y, z: interpPos.z },
//       rotation: {
//         x: interpQuat.x,
//         y: interpQuat.y,
//         z: interpQuat.z,
//         w: interpQuat.w,
//       },
//     });
//     idx += 1;
//   }
//   return goals;
// };

// const createJointGoals = (jointState1, jointState2, duration, jointNames) => {
//   const numGoals = Math.ceil(duration / FRAME_TIME);
//   let goals = [];
//   let idx = 0;
//   let tmpJointGoals = {};
//   // console.log("createJointGoals",{tmpJointGoals,jointState1,jointState2})
//   while (idx <= numGoals) {
//     const percent = idx / numGoals;
//     // console.log('percent',percent)
//     jointNames.forEach((jointKey) => {
//       tmpJointGoals[jointKey] =
//         (1 - percent) * jointState1[jointKey] + percent * jointState2[jointKey];
//       // if (jointKey === 'wrist_2_joint') {
//       // console.log("createJointGoalsInner",{percent,joint1:jointState1[jointKey],joint2:jointState2[jointKey],interp:tmpJointGoals[jointKey]})
//       // }
//       // console.log("createJointGoalsInner",{tmpJointGoals,jointState1,jointState2,percent,jointKey,joint1:jointState1[jointKey],joint2:jointState2[jointKey],interp:tmpJointGoals[jointKey]})
//     });
//     goals.push({
//       values: [
//         null,
//         null,
//         ...jointNames.map((joint) => ({ Scalar: tmpJointGoals[joint] })),
//       ],
//       weights: [
//         SMOOTHNESS_WEIGHT,
//         COLLISION_WEIGHT,
//         ...jointNames.map(() => 20),
//       ],
//       joints: cloneDeep(tmpJointGoals),
//     });
//     idx += 1;
//   }
//   // console.log('GOALS',goals)
//   return goals;
// };

// const createIKSensitivityTester = (
//   attachmentLink,
//   jointNames,
//   startJoints,
//   endJoints,
//   duration,
//   type1,
//   type2
// ) => {
//   const tester = (state, goal, percent) => {
//     // return true;
//     const p = state.frames[attachmentLink].world.translation;
//     const r = state.frames[attachmentLink].world.rotation;
//     const achievedPos = { x: p[0], y: p[1], z: p[2] };
//     const goalQuat = new Quaternion(
//       goal.rotation.x,
//       goal.rotation.y,
//       goal.rotation.z,
//       goal.rotation.w
//     );
//     const achievedQuat = new Quaternion(r[0], r[1], r[2], r[3]);
//     const translationalDistance = distance(achievedPos, goal.position);
//     const rotationalDistance = goalQuat.angleTo(achievedQuat);
//     console.log("percent", percent);
//     // const sensitivity = type1 === 'location' && type2 === 'location'
//     //   ?
//     const translationalDistanceLimit = 0.03;
//     const rotationalDistanceLimit = 0.03;
//     // const translationalDistanceLimit = Math.max(
//     //   0.01,
//     //   timeGradientFunction(
//     //     idx,
//     //     0.01,
//     //     0.6,
//     //     Math.min(duration / 10, 30),
//     //     duration
//     //   )
//     // );
//     // const rotationalDistanceLimit = Math.max(
//     //   0.01,
//     //   timeGradientFunction(idx, 0.01, 4, Math.min(duration / 10, 30), duration)
//     // );
//     let passed =
//       translationalDistance <= translationalDistanceLimit &&
//       rotationalDistance <= rotationalDistanceLimit;
//     // console.log()
//     if (!passed) {
//       // console.log('POS NOT PASSED',{ passed, translationalDistance, rotationalDistance, translationalDistanceLimit, rotationalDistanceLimit })
//       return false;
//     }
//     return true;
//     return !jointNames.some((jointName) => {
//       const jointDistanceStart = Math.abs(
//         state.joints[jointName] - startJoints[jointName]
//       );
//       const jointDistanceEnd = Math.abs(
//         state.joints[jointName] - endJoints[jointName]
//       );
//       const jointDistanceLimitStart = timeGradientFunctionOneTailStart(
//         idx,
//         0.05,
//         4 * Math.PI,
//         Math.min(duration / 10, 30)
//       );
//       const jointDistanceLimitEnd = timeGradientFunctionOneTailEnd(
//         idx,
//         0.05,
//         4 * Math.PI,
//         Math.min(duration / 10, 30),
//         duration
//       );
//       const jointPassed =
//         jointDistanceStart < jointDistanceLimitStart &&
//         jointDistanceEnd < jointDistanceLimitEnd;
//       if (!jointPassed) {
//         console.log("JOINT NOT PASSED", {
//           jointName,
//           passed: jointPassed,
//           jointDistanceStart,
//           jointDistanceLimitStart,
//           jointDistanceEnd,
//           jointDistanceLimitEnd,
//         });
//       }
//       return !jointPassed;
//     });
//   };

//   return tester;
// };

// const createJointSensitivityTester = (jointNames) => {
//   const tester = (state, goal, _) => {
//     // return true
//     return !jointNames.some((jointName) => {
//       const passed =
//         Math.abs(state.joints[jointName] - goal.joints[jointName]) < 0.1;
//       // if (!passed) {console.log('NOT PASSED',{jointName, dist: Math.abs(state.joints[jointName] - goal.joints[jointName]) , state: state.joints[jointName], goal: goal.joints[jointName]})}
//       return !passed;
//     });
//   };

//   return tester;
// };

// const stepIKSimple = ({
//   currentPose,
//   currentJoints,
//   goalPose,
//   goalJoints,
//   solver,
//   speed,
//   joints,
//   attachmentLink,
//   poseType,
// }) => {
//   const currentPosition = new Vector3(
//     currentPose.position.x,
//     currentPose.position.y,
//     currentPose.position.z
//   );

//   // const currentQuaternion = new Quaternion(
//   //   currentPose.rotation.x,
//   //   currentPose.rotation.y,
//   //   currentPose.rotation.z,
//   //   currentPose.rotation.w
//   // );

//   const goalPosition = new Vector3(
//     goalPose.position.x,
//     goalPose.position.y,
//     goalPose.position.z
//   );

//   // const goalQuaternion = new Quaternion(
//   //   goalPose.rotation.x,
//   //   goalPose.rotation.y,
//   //   goalPose.rotation.z,
//   //   goalPose.rotation.w
//   // );

//   // const interpPos = new Vector3(0, 0, 0);
//   // const interpQuat = new Quaternion(0, 0, 0, 1);

//   const posDist = currentPosition.distanceTo(goalPosition);
//   // const rotDist = currentQuaternion.angleTo(goalQuaternion);

//   // const jointDists = mapValues(currentJoints, (v, key) =>
//   //   Math.abs(v - goalJoints[key])
//   // );

//   // console.log({ posDist, rotDist, jointDists });

//   let passed =
//     poseType === "waypoint"
//       ? posDist <= MAX_POSE_DISTANCE_DIFF * 2
//       : !Object.values(jointDists).some((v) => v >= MAX_JOINT_DISTANCE_DIFF);
//   if (passed) {
//     return { newState: solver.currentState, improved: true, reached: true };
//   }

//   // if (posDist < speed/20) {
//   //   interpPos.copy(goalPosition)
//   // } else {
//   //   interpPos
//   //     .copy(goalPosition)
//   //     .sub(currentPosition)
//   //     .clampLength(0, speed / 15)
//   //     .add(currentPosition);
//   // }
//   // if (rotDist < speed/20) {
//   //   interpQuat.copy(goalQuaternion);
//   // } else {
//   //   interpQuat.copy(currentQuaternion).rotateTowards(goalQuaternion, speed / 15);
//   // }

//   // const nextJoints = mapValues(currentJoints, (currentJointValue, jointKey) => {
//   //   const signedDist = goalJoints[jointKey] - currentJointValue;
//   //   const direction = signedDist / Math.abs(signedDist);
//   //   const travel = (direction * speed) / 5;
//   //   if (Math.abs(signedDist) < Math.abs(travel)) {
//   //     return goalJoints[jointKey];
//   //   } else {
//   //     // console.log(travel)
//   //     return currentJointValue + travel;
//   //   }
//   // });

//   let goals = {
//     eePosition: {
//       Translation: [goalPosition.x, goalPosition.y, goalPosition.z],
//     },
//     // eeRotation: { Rotation: [goalQuaternion.x, goalQuaternion.y, goalQuaternion.z, goalQuaternion.w] }
//   };
//   joints.forEach((j) => {
//     goals[j] = { Scalar: goalJoints[j] };
//   });

//   let weights = {
//     smoothness: 50,
//     collision: 5,
//     eePosition: 35,
//     // eeRotation:0
//   };
//   joints.forEach((j) => {
//     weights[j] = poseType === "waypoint" ? 0 : 5;
//   });

//   const newState = solver.solve(goals, weights);

//   const newPoseRaw = newState.frames[attachmentLink].world;
//   const newPosition = new Vector3(
//     newPoseRaw.translation[0],
//     newPoseRaw.translation[1],
//     newPoseRaw.translation[2]
//   );
//   // const newQuaternion = new Quaternion(
//   //   newPoseRaw.rotation[0],
//   //   newPoseRaw.rotation[1],
//   //   newPoseRaw.rotation[2],
//   //   newPoseRaw.rotation[3]
//   // );

//   // console.log({
//   //   interpPos,
//   //   interpQuat,
//   //   speed,
//   //   currentPosition,
//   //   currentQuaternion,
//   //   nextJoints,
//   //   goalPosition,
//   //   goalQuaternion,
//   //   newPosition,
//   // });

//   const hasMovement = joints.some((jointKey) => {
//     const delta = Math.abs(newState.joints[jointKey] - currentJoints[jointKey]);
//     if (delta > 0.005) {
//       return true;
//     }
//     return false;
//   });

//   // const newPoseError = goalPosition.distanceTo(newPosition);// + goalQuaternion.angleTo(newQuaternion);
//   // const newJointError = joints
//   // .map((jointKey) =>
//   //   Math.abs(newState.joints[jointKey] - goalJoints[jointKey])
//   // )
//   // .reduce((next, current) => next + current);

//   // const oldPoseError = posDist;// + rotDist;
//   // const oldJointError = joints
//   // .map((jointKey) => jointDists[jointKey])
//   // .reduce((next, current) => next + current);

//   // console.log({ newPoseError, oldPoseError, newJointError, oldJointError });

//   // const improved =  oldPoseError - newPoseError > 0.0001;//|| oldJointError - newJointError > 0.0001 ;

//   return { newState, improved: hasMovement, reached: false };
// };

// const stepJointSimple = ({
//   currentJoints,
//   goalJoints,
//   solver,
//   speed,
//   joints,
// }) => {
//   const jointDists = mapValues(currentJoints, (v, key) =>
//     Math.abs(v - goalJoints[key])
//   );
//   // console.log({currentJoints,goalJoints,jointDists})
//   console.log("running simple", jointDists);
//   let passed = !Object.values(jointDists).some(
//     (v) => v >= MAX_JOINT_DISTANCE_DIFF
//   );
//   if (passed) {
//     return { newState: solver.currentState, improved: true, reached: true };
//   }

//   // console.log('nextJoints',nextJoints)
//   console.log("goals/objectives", {
//     goals: solver.currentGoals,
//     objectives: solver.objectives,
//   });

//   let goals = {};
//   joints.forEach((j) => {
//     goals[j] = { Scalar: goalJoints[j] };
//   });

//   let weights = { smoothness: SIMPLE_SMOOTHNESS_WEIGHT, collision: 0 };
//   joints.forEach((j) => {
//     weights[j] = SIMPLE_JOINT_WEIGHT;
//   });

//   const newState = solver.solve(goals, weights);

//   const hasMovement = joints.some((jointKey) => {
//     const delta = Math.abs(newState.joints[jointKey] - currentJoints[jointKey]);
//     if (delta > 0.01) {
//       return true;
//     }
//     return false;
//   });

//   // const newError = joints
//   //   .map((jointKey) =>
//   //     Math.abs(newState.joints[jointKey] - goalJoints[jointKey])
//   //   )
//   //   .reduce((next, current) => next + current);
//   // const oldError = joints
//   //   .map((jointKey) => jointDists[jointKey])
//   //   .reduce((next, current) => next + current);

//   // console.log({ newError, oldError });

//   // const improved = newError < oldError;

//   return { newState, improved: hasMovement, reached: false };
// };