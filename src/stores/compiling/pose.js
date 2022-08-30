import { STATUS, ROOT_PATH, ERROR, MAX_POSE_DISTANCE_DIFF, MAX_POSE_ROTATION_DIFF } from "../Constants";
import { Quaternion } from "three";
import { likStateToData } from "../../helpers/conversion";
import {
//   createStaticEnvironment,
  queryWorldPose,
  // quaternionLog,
  eulerFromQuaternion,
  poseToGoalPosition,
  distance,
} from "../../helpers/geometry";
// import { DATA_TYPES } from "simple-vp";
import { range, random } from "lodash";

const COLLISION_WEIGHT = 5;
const SMOOTHNESS_WEIGHT = 0.1;

const sampleJoints = (joints) => {
  let jointState = {};
  joints.forEach((joint) => {
    if (joint.type !== "fixed") {
      jointState[joint.name] = random(joint.lowerBound, joint.upperBound, true);
    }
  });
  return jointState;
};

export const poseCompiler = ({
  data,
  properties,
  path,
  memo,
  module,
  worldModel,
}) => {
  // Create a reachabilty object to indicate who can reach this pose;
  let reachability = {};
  let states = {};
  let goalPose = null;
  let status = STATUS.VALID;
  let errorCode = null;

  // Enumerate the robotAgentTypes/gripperTypes currently in the memo. This is technically unsafe,
  // but we pre-process them beforehand so it is fine. We also always assume root execution
  // (which is fine for robots/humans/grippers).

  // console.log('running pose compiler')

  const grippers = Object.values(memo).filter((v) => v.type === "gripperType");

  const staticEnvironment = []; // createStaticEnvironment(worldModel);

  Object.values(memo)
    .filter((v) => v.type === "robotAgentType")
    .forEach((robot) => {
      reachability[robot.id] = {};
      states[robot.id] = {};

      // Retrieve the robot's position/orientation
      const basePose = queryWorldPose(worldModel, robot.id);
      // const quatLog = quaternionLog(basePose.rotation);
      const baseEuler = eulerFromQuaternion([basePose.w, basePose.x, basePose.y, basePose.z],'sxyz');
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
      const urdf = robot.properties.compiled[ROOT_PATH].urdf;
      const robotInitialJointState =
        robot.properties.compiled[ROOT_PATH].initialJointState;

      const proximity = robot.properties.compiled[ROOT_PATH].proximity

      // Enumerate grippers and search for ones that are based on this robot.
      grippers.forEach((gripper) => {
        if (robotLinks.includes(gripper.properties.relativeTo)) {
          // Check to see if there are previously calculated joint values for this pose.
          const initialJointState = data.properties.compiled?.[path]?.states[
            robot.id
          ]?.[gripper.id]?.joints
            ? data.properties.compiled[path].states[robot.id][gripper.id].joints
            : robotInitialJointState;

          // Set up the objectives based on the attachment link
          const attachmentLink = gripper.properties.relativeTo;
          const objectives = [
            {
              type: "PositionMatch",
              name: "EE Position",
              link: attachmentLink,
              weight: 50,
            },
            {
              type: "OrientationMatch",
              name: "EE Rotation",
              link: attachmentLink,
              weight: 25,
            },
            {
              type: "CollisionAvoidance",
              name: "Collision Avoidance",
              weight: COLLISION_WEIGHT,
            },
            {
              type: "PositionMatch",
              name: "Mid-Chain Up",
              link: "wrist_1_link",
              weight: 5,
            },
            { type: "SmoothnessMacro", name: "General Smoothness", weight: SMOOTHNESS_WEIGHT },
          ];

          // Find the position we need in the attachment link to match the desired pose gripper position
          goalPose = poseToGoalPosition(
            worldModel,
            gripper.id,
            attachmentLink,
            properties
          );
          // console.log('goal:',goalPose)

          // Construct the joint state information
          let state = {};

          // Construct the solver
          const initialState = { origin, joints: initialJointState, proximity };
          const solver = new module.Solver(
            urdf,
            objectives,
            rootBounds,
            staticEnvironment,
            initialState,
            false,
            1,
            150
          );

          // console.log('proximity',proximity)

          // Construct the goals
          const pos = goalPose.position;
          const rot = goalPose.rotation;
          const goalQuat = new Quaternion(rot.x, rot.y, rot.z, rot.w);
          const goals = [
            { Translation: [pos.x, pos.y, pos.z] },
            { Rotation: [rot.x, rot.y, rot.z, rot.w] },
            null,
            { Translation: [0, 0, 1] },
            null,
          ];
          let goalAchieved = false;

          let restarts = range(0, 20);
          let rounds = range(0, 10);

          restarts.some(() => {
            // let currentTime = Date.now();
            rounds.some(() => {
              state = solver.solve(goals, [50, 30, COLLISION_WEIGHT, 5, SMOOTHNESS_WEIGHT]);
              const p = state.frames[attachmentLink].world.translation;
              const r = state.frames[attachmentLink].world.rotation;
              const achievedPos = { x: p[0], y: p[1], z: p[2] };
              const achievedQuat = new Quaternion(r[0], r[1], r[2], r[3]);
              const translationDistance = distance(achievedPos, pos);
              const rotationalDistance = goalQuat.angleTo(achievedQuat);
              // console.log({translationDistance,rotationalDistance})
              if (translationDistance < MAX_POSE_DISTANCE_DIFF && rotationalDistance < MAX_POSE_ROTATION_DIFF) {
                goalAchieved = true;
              }
              // if (translationDistance < 0.01) {
              //     goalAchieved = true
              // }
              return goalAchieved;
            });
            if (!goalAchieved) {
              let newStart = sampleJoints(solver.joints);
              // console.warn(newStart)
              solver.reset({ origin, joints: newStart }, [50, 30, COLLISION_WEIGHT, 5, SMOOTHNESS_WEIGHT]);
            }
            return goalAchieved;
          });

          if (!goalAchieved) {
            status = STATUS.WARN;
            errorCode = ERROR.UNREACHABLE_POSE;
          }
          reachability[robot.id][gripper.id] = goalAchieved;
          // console.log(robot.properties.compiled[ROOT_PATH].linkParentMap);
          states[robot.id][gripper.id] = likStateToData(state, robot.id, robot.properties.compiled[ROOT_PATH].linkParentMap);
          console.log(states[robot.id][gripper.id].proximity)
          // console.log(states[robot.id][gripper.id])
          // states[robot.id][gripper.id] = likStateToData(
          //   state,
          //   worldModel,
          //   robot.id
          // );
          // delete solver;
        }
      });
    });

  // console.log("pose recalculation: ", { data, reachability, states });

  const newCompiled = {
    goalPose,
    states,
    reachability,
    status,
    errorCode,
    otherPropertyUpdates: { states, reachability },
  };
  return newCompiled;
};
