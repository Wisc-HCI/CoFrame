import * as Comlink from "comlink";
import init, { computePose } from "coframe-rust";

// const forward = async ({ urdf, joints, origin, staticEnvironment }) => {
//   console.log("IN FORWARD");
//   const baseEuler = quaternionToEuler(origin.rotation);

//   console.log("CREATED EULER");

//   await init();

//   console.log("PASSED INIT");

//   let solver = new Solver(
//     urdf,
//     {
//       collision: {
//         name: "CollisionAvoidance",
//         type: "CollisionAvoidance",
//         weight: 3,
//       },
//     },
//     [
//       { value: origin.translation[0], delta: 0.0 },
//       { value: origin.translation[1], delta: 0.0 },
//       { value: origin.translation[2], delta: 0.0 }, // Translational
//       { value: baseEuler.x, delta: 0.0 },
//       { value: baseEuler.y, delta: 0.0 },
//       { value: baseEuler.z, delta: 0.0 }, // Rotational
//     ],
//     staticEnvironment
//   );

//   console.log("solver", solver);
//   solver.computeAverageDistanceTable();

//   return solver.forward({
//     origin,
//     joints,
//   });
// };

const inverse = async ({
  urdf,
  gripperGoalPose,
  origin,
  attachmentLink,
  staticEnvironment,
}) => {
  await init();
  return computePose(
    urdf,
    gripperGoalPose,
    origin,
    attachmentLink,
    staticEnvironment
  );
};

Comlink.expose({
  inverse,
});
