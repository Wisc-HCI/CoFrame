import { STATUS, STEP_TYPE, ROOT_PATH, ERROR } from "../Constants";
import {
  timeGradientFunction,
  timeGradientFunctionOneTailStart,
  timeGradientFunctionOneTailEnd,
} from "../helpers";
import { likStateToData } from "../../helpers/conversion";
import {
  createStaticEnvironment,
  queryWorldPose,
  quaternionLog,
  eulerFromQuaternion,
  distance,
  attachmentToEEPose,
} from "../../helpers/geometry";
import { merge, cloneDeep, zipObject } from "lodash";
import { Quaternion, Vector3 } from "three";
import { eventsToStates, statesToSteps } from ".";

const FRAME_TIME = 30;
const POSITION_WEIGHT = 30;
const ROTATION_WEIGHT = 0;
const COLLISION_WEIGHT = 5;
const SMOOTHNESS_WEIGHT = 50;
const IMPROVEMENT_THRESHOLD = 0.5;
const MS_TIME_LIMIT = 7000;

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

const createIKGoals = (
  goalPose1,
  goalPose2,
  jointState1,
  jointState2,
  duration,
  jointNames
) => {
  const numGoals = Math.ceil(duration / FRAME_TIME);
  let goals = [];
  let idx = 0;
  const goalPos1 = new Vector3(
    goalPose1.position.x,
    goalPose1.position.y,
    goalPose1.position.z
  );
  const goalPos2 = new Vector3(
    goalPose2.position.x,
    goalPose2.position.y,
    goalPose2.position.z
  );
  const goalQuat1 = new Quaternion(
    goalPose1.rotation.x,
    goalPose1.rotation.y,
    goalPose1.rotation.z,
    goalPose1.rotation.w
  );
  const goalQuat2 = new Quaternion(
    goalPose2.rotation.x,
    goalPose2.rotation.y,
    goalPose2.rotation.z,
    goalPose2.rotation.w
  );
  const interpPos = new Vector3(0, 0, 0);
  const interpQuat = new Quaternion(0, 0, 0, 1);
  let jointGoals = { ...jointState1 };
  while (idx <= numGoals) {
    const percent = idx / numGoals;
    interpPos.lerpVectors(goalPos1, goalPos2, percent);
    interpQuat.slerpQuaternions(goalQuat1, goalQuat2, percent);
    jointNames.forEach((jointKey) => {
      jointGoals[jointKey] =
        percent * jointState2[jointKey] + (1 - percent) * jointState1[jointKey];
    });
    const startJointGoalWeight = timeGradientFunctionOneTailStart(
      idx,
      10,
      -20,
      Math.min(numGoals / 5, 30)
    );
    const endJointGoalWeight = timeGradientFunctionOneTailEnd(
      idx,
      10,
      -20,
      Math.min(numGoals / 5, 30),
      numGoals
    );
    goals.push({
      values: {
        eePosition: { Translation: [interpPos.x, interpPos.y, interpPos.z] },
        eeRotation: { Rotation: [interpQuat.x, interpQuat.y, interpQuat.z, interpQuat.w] },
        ...zipObject(jointNames,jointNames.map((joint) => ({ Scalar: jointState2[joint] })))
      },
      weights: zipObject(jointNames,jointNames.map(() => endJointGoalWeight)),
      joints: jointGoals,
      position: { x: interpPos.x, y: interpPos.y, z: interpPos.z },
      rotation: {
        x: interpQuat.x,
        y: interpQuat.y,
        z: interpQuat.z,
        w: interpQuat.w,
      },
    });
    idx += 1;
  }
  return goals;
};

const createJointGoals = (jointState1, jointState2, duration, jointNames) => {
  const numGoals = Math.ceil(duration / FRAME_TIME);
  let goals = [];
  let idx = 0;
  let tmpJointGoals = {};
  // console.log("createJointGoals",{tmpJointGoals,jointState1,jointState2})
  while (idx <= numGoals) {
    const percent = idx / numGoals;
    // console.log('percent',percent)
    jointNames.forEach((jointKey) => {
      tmpJointGoals[jointKey] =
        (1 - percent) * jointState1[jointKey] + percent * jointState2[jointKey];
      // if (jointKey === 'wrist_2_joint') {
      // console.log("createJointGoalsInner",{percent,joint1:jointState1[jointKey],joint2:jointState2[jointKey],interp:tmpJointGoals[jointKey]})
      // }
      // console.log("createJointGoalsInner",{tmpJointGoals,jointState1,jointState2,percent,jointKey,joint1:jointState1[jointKey],joint2:jointState2[jointKey],interp:tmpJointGoals[jointKey]})
    });
    goals.push({
      values: [
        null,
        null,
        ...jointNames.map((joint) => ({ Scalar: tmpJointGoals[joint] })),
      ],
      weights: [COLLISION_WEIGHT, 7, ...jointNames.map(() => 20)],
      joints: cloneDeep(tmpJointGoals),
    });
    idx += 1;
  }
  // console.log('GOALS',goals)
  return goals;
};

const createIKSensitivityTester = (
  attachmentLink,
  jointNames,
  startJoints,
  endJoints,
  duration
) => {
  const tester = (state, goal, idx) => {
    // return true;
    const p = state.frames[attachmentLink].world.translation;
    const r = state.frames[attachmentLink].world.rotation;
    const achievedPos = { x: p[0], y: p[1], z: p[2] };
    const goalQuat = new Quaternion(
      goal.rotation.x,
      goal.rotation.y,
      goal.rotation.z,
      goal.rotation.w
    );
    const achievedQuat = new Quaternion(r[0], r[1], r[2], r[3]);
    const translationalDistance = distance(achievedPos, goal.position);
    const rotationalDistance = goalQuat.angleTo(achievedQuat);
    const translationalDistanceLimit = Math.max(
      0.01,
      timeGradientFunction(
        idx,
        0.01,
        0.6,
        Math.min(duration / 10, 30),
        duration
      )
    );
    const rotationalDistanceLimit = Math.max(
      0.01,
      timeGradientFunction(idx, 0.01, 4, Math.min(duration / 10, 30), duration)
    );
    let passed =
      translationalDistance <= translationalDistanceLimit &&
      rotationalDistance <= rotationalDistanceLimit;
    // console.log()
    if (!passed) {
      // console.log('POS NOT PASSED',{ passed, translationalDistance, rotationalDistance, translationalDistanceLimit, rotationalDistanceLimit })
      return false;
    }
    return true;
    return !jointNames.some((jointName) => {
      const jointDistanceStart = Math.abs(
        state.joints[jointName] - startJoints[jointName]
      );
      const jointDistanceEnd = Math.abs(
        state.joints[jointName] - endJoints[jointName]
      );
      const jointDistanceLimitStart = timeGradientFunctionOneTailStart(
        idx,
        0.05,
        4 * Math.PI,
        Math.min(duration / 10, 30)
      );
      const jointDistanceLimitEnd = timeGradientFunctionOneTailEnd(
        idx,
        0.05,
        4 * Math.PI,
        Math.min(duration / 10, 30),
        duration
      );
      const jointPassed =
        jointDistanceStart < jointDistanceLimitStart &&
        jointDistanceEnd < jointDistanceLimitEnd;
      if (!jointPassed) {
        console.log("JOINT NOT PASSED", {
          jointName,
          passed: jointPassed,
          jointDistanceStart,
          jointDistanceLimitStart,
          jointDistanceEnd,
          jointDistanceLimitEnd,
        });
      }
      return !jointPassed;
    });
  };

  return tester;
};

const createJointSensitivityTester = (jointNames) => {
  const tester = (state, goal, _) => {
    // return true
    return !jointNames.some((jointName) => {
      const passed =
        Math.abs(state.joints[jointName] - goal.joints[jointName]) < 0.1;
      // if (!passed) {console.log('NOT PASSED',{jointName, dist: Math.abs(state.joints[jointName] - goal.joints[jointName]) , state: state.joints[jointName], goal: goal.joints[jointName]})}
      return !passed;
    });
  };

  return tester;
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
    trajectory.properties.compiled[path].startLocation.properties.compiled[
      path
    ],
    ...trajectory.properties.compiled[path].waypoints.map(
      (wp) => wp.properties.compiled[path]
    ),
    trajectory.properties.compiled[path].endLocation.properties.compiled[path],
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

        let reached = true;

        // Instantiate the initial state and solver;
        let objectives = {
          smoothness: {
            type: "SmoothnessMacro",
            name: "Smoothness",
            weight: SMOOTHNESS_WEIGHT,
          },
          collision: {
            type: "CollisionAvoidance",
            name: "Collision Avoidance",
            weight: 7,
          },
        };

        if (motionType === "IK") {
          objectives["eePosition"] = {
            type: "PositionMatch",
            name: "EE Position",
            link: attachmentLink,
            weight: POSITION_WEIGHT,
          };
          objectives["eeRotation"] = {
            type: "OrientationMatch",
            name: "EE Rotation",
            link: attachmentLink,
            weight: ROTATION_WEIGHT,
          };
        }
        jointNames.forEach(
          (jointKey) =>
            (objectives[jointKey] = {
              type: "JointMatch",
              name: `JointControl:${jointKey}`,
              joint: jointKey,
              weight: motionType === "IK" ? 0 : 20,
            })
        );

        // Find the position we need in the attachment link to match the desired pose gripper position

        // console.log('goal:',goalPose)

        // Construct the joint state information
        let state = {};

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
        let mainIdx = 0;
        while (poseStack.length > 1 && reached) {
          const pose1 = poseStack[0];
          const pose2 = poseStack[1];
          // console.log('pose stacks', { pose1, pose2 });
          const goalPose1 = pose1.goalPose;
          const goalPose2 = pose2.goalPose;
          const jointState1 = pose1.states[robot.id][gripper.id].joints;
          const jointState2 = pose2.states[robot.id][gripper.id].joints;
          // console.log({jointState1,jointState2})

          const dist = distance(goalPose1.position, goalPose2.position);
          const duration = (1000 * dist) / velocity;

          // console.log('creating goals');
          let goals =
            motionType === "IK"
              ? createIKGoals(
                  goalPose1,
                  goalPose2,
                  jointState1,
                  jointState2,
                  duration,
                  jointNames
                )
              : createJointGoals(
                  jointState1,
                  jointState2,
                  duration,
                  jointNames
                );

          // console.log('creating sensitivity testers')
          const sensitivityTester =
            motionType === "IK"
              ? createIKSensitivityTester(
                  attachmentLink,
                  jointNames,
                  jointState1,
                  jointState2,
                  goals.length
                )
              : createJointSensitivityTester(jointNames);

          let idx = 0;

          while (goals.length > 0 && reached) {
            // console.log('animation frames: ', innerSteps.length)
            // console.log('goals: ', goals.length)
            const goal = goals[0];
            // console.log(goal)
            state = solver.solve(goal.values, goal.weights);
            reached = sensitivityTester(state, goal, idx);

            // const stateData = likStateToData(state, robot.properties.root);
            // console.log(robot.properties.compiled[ROOT_PATH].linkParentMap);
            const stateData = likStateToData(
              state,
              robot.id,
              robot.properties.compiled[ROOT_PATH].linkParentMap
            );
            if (!reached && status !== STATUS.FAILED) {
              status = STATUS.WARN;
              errorCode = ERROR.TRAJECTORY_PROGRESS;
            }

            // Calculate the eePose of the gripper
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
            idx += 1;
            mainIdx += 1;
            goals.shift();
          }
          // Pop off the first item in the poseStack
          poseStack.shift();
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

  const events = [
    {
      condition: robot
        ? {
            [robot.id]: { busy: false },
          }
        : {},
      onTrigger: [initialStep, ...innerSteps, finalStep],
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
