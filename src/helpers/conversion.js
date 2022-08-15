import {mapValues} from "lodash";
import { queryLocalPose, DEFAULT_SCALE } from "./geometry";
import { Quaternion, Vector3, Matrix4 } from "three";

export const transformToThreeMatrix = (transform) => {
    const matrix = new Matrix4();
    matrix.compose(
      positionToThreeVector(transform.position),
      rotationToThreeQuaternion(transform.rotation),
      DEFAULT_SCALE
    );
    return matrix;
  };
  
  export const threeMatrixToTransform = (matrix) => {
    const positionVector = new Vector3();
    const rotationQuat = new Quaternion();
    positionVector.setFromMatrixPosition(matrix);
    rotationQuat.setFromRotationMatrix(matrix);
    return {
      position: {
        x: positionVector.x,
        y: positionVector.y,
        z: positionVector.z,
      },
      rotation: {
        x: rotationQuat.x,
        y: rotationQuat.y,
        z: rotationQuat.z,
        w: rotationQuat.w,
      },
      scale: {
        x: 1,
        y: 1,
        z: 1,
      },
    };
  };
  
  export const positionToThreeVector = (position) =>
    new Vector3(position.x, position.y, position.z);
  
  export const threeVectorToPosition = (vector) => ({
    x: vector.x,
    y: vector.y,
    z: vector.z,
  });
  
  export const threeQuaternionToRotation = (quaternion) => ({
    x: quaternion.x,
    y: quaternion.y,
    z: quaternion.z,
    w: quaternion.w,
  });
  
  export const rotationToThreeQuaternion = (rotation) =>
    new Quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
  


export const likProximityAdjustment = (trackedPoints, proximity, mirrorTrackedPoints) => {
    let trackedPinchPoints = trackedPoints ? trackedPoints : [];
    let proximityModel = {};
  
    if (!proximity) {
      return proximityModel;
    }
    proximity.forEach(({ shape1, shape2, distance, points, physical }) => {
      if (trackedPinchPoints !== []) { 
        trackedPinchPoints.forEach(({ link1, link2 }) => {
          if (
            (link1 === shape1 && link2 === shape2) ||
            (link2 === shape1 && link1 === shape2)
          ) {
            if (!proximityModel[link1]) {
              proximityModel[link1] = {};
            }
            if (mirrorTrackedPoints && !proximityModel[link2]) {
              proximityModel[link2] = {};
            }
            proximityModel[link1][link2] = {
              distance: distance,
              physical: physical,
              points: points,
            };
            if (mirrorTrackedPoints) {
              proximityModel[link2][link1] = {
                distance: distance,
                physical: physical,
                points: points,
              };
            }
          }
        });
      } else {
        if (!proximityModel[link1]) {
          proximityModel[link1] = {};
        }
        if (!proximityModel[link2]) {
          proximityModel[link2] = {};
        }
        proximityModel[link1][link2] = {
          distance: distance,
          physical: physical,
          points: points,
        };
        proximityModel[link2][link1] = {
          distance: distance,
          physical: physical,
          points: points,
        };
      }
    });
  
    return proximityModel;
  };
  
  export const likFramesToTransforms = (frames, model, frame) => {
    const relativeFrameId = frame ? frame : "world";
    const linkTransforms = mapValues(frames, (frameData) => {
      const poseWorld = {
        position: {
          x: frameData.world.translation[0],
          y: frameData.world.translation[1],
          z: frameData.world.translation[2],
        },
        rotation: {
          w: frameData.world.rotation[3],
          x: frameData.world.rotation[0],
          y: frameData.world.rotation[1],
          z: frameData.world.rotation[2],
        },
      };
      const poseLocal = queryLocalPose(model, relativeFrameId, poseWorld);
      return {
        frame,
        ...poseLocal,
        scale: {
          x: 1,
          y: 1,
          z: 1,
        },
      };
    });
    return linkTransforms;
  };
  
  export const likStateToData = (state, model, frame) => {
    // console.log('PRE-PARSED STATE DATA',state);
    // console.log({frames:state.frames,model,frame})
    const data = {
      joints: state.joints,
      links: likFramesToTransforms(state.frames, model, frame),
      proximity: state.proximity,
    };
    // console.log('PARSED STATE DATA',data)
    return data;
  };