import lodash from "lodash";
// import { GRIPPER_CONFIGURATIONS, GRIPPER_FRAMES, GRIPPER_PARENTS } from './gripper';
import { Quaternion, Vector3, Group, Object3D, Matrix4 } from "three";
import { ConvexGeometry } from "three-stdlib";
// import { EVD_MESH_LOOKUP } from './initialSim';
import { DATA_TYPES } from "simple-vp";
import {
  HAND_PINCH_MAX_DISTANCE,
  HAND_PINCH_MIN_DISTANCE,
  REFERENCEABLE_OBJECTS,
  STEP_TYPE,
} from "./Constants";
// import { merge } from "lodash";

Object3D.DefaultUp.set(0, 0, 1);

export const distance = (pos1, pos2) => {
  return Math.sqrt(
    Math.pow(pos1.x - pos2.x, 2) +
      Math.pow(pos1.y - pos2.y, 2) +
      Math.pow(pos1.z - pos2.z, 2)
  );
};

// https://www.desmos.com/calculator/0rd3tessqn
/* 
Computes a domed curve that has a fixed increase/decrease period and max value. 
*/
export const timeGradientFunction = (
  t,
  heightOffset,
  maxHeight,
  slope,
  windowLength
) => {
  const val = Math.min(
    timeGradientFunctionOneTailStart(t, heightOffset, maxHeight, slope),
    timeGradientFunctionOneTailEnd(
      t,
      heightOffset,
      maxHeight,
      slope,
      windowLength
    )
  );
  // console.log({val,t,heightOffset,maxHeight,slope,windowLength})
  return val;
};

/* 
Similar to above, but the minimum value occurs at the start, and maximizes forever after. 
*/
export const timeGradientFunctionOneTailStart = (
  t,
  heightOffset,
  maxHeight,
  slope
) => {
  return (
    maxHeight / (1 + Math.exp((-1 * t) / slope)) - maxHeight / 2 + heightOffset
  );
};

/* 
Similar to above, but the minimum value occurs at the end, and maximizes beforehand. 
*/
export const timeGradientFunctionOneTailEnd = (
  t,
  heightOffset,
  maxHeight,
  slope,
  windowLength
) => {
  return (
    maxHeight / (1 + Math.exp((t - windowLength) / slope)) -
    maxHeight / 2 +
    heightOffset
  );
};

export const quaternionLog = (quaternion) => {
  let outVec = new Vector3(quaternion.x, quaternion.y, quaternion.z);
  if (Math.abs(quaternion.w) < 1.0) {
    let a = Math.acos(quaternion.w);
    let sina = Math.sin(a);
    if (Math.abs(sina) >= 0.005) {
      let c = a / sina;
      outVec.multiplyScalar(c);
    }
  }
  return [outVec.x, outVec.y, outVec.z];
};

export const DEFAULT_LOCATION_COLOR = { r: 62, g: 16, b: 102, a: 1 };

export const DEFAULT_WAYPOINT_COLOR = { r: 100, g: 18, b: 128, a: 1 };

export const DEFAULT_TRAJECTORY_COLOR = { r: 209, g: 0, b: 146, a: 1 };

export const UNREACHABLE_COLOR = { r: 204, g: 75, b: 10, a: 1 };

export const OCCUPANCY_ERROR_COLOR = { r: 233, g: 53, b: 152, a: 1 };

export const updateWorldModelWithTransforms = (model, transforms) => {
  Object.keys(transforms).forEach((transformId) => {
    const transform = transforms[transformId];
    model[transformId].position.set(
      transform.position.x,
      transform.position.y,
      transform.position.z
    );
    model[transformId].quaternion.set(
      transform.rotation.x,
      transform.rotation.y,
      transform.rotation.z,
      transform.rotation.w
    );
  });
  return model;
};

export const computeRelativePose = (model, referenceId, targetId) => {
  const targetPose = queryWorldPose(model, targetId);
  // console.log({referenceId,targetId,targetPose});
  return queryLocalPose(model, referenceId, targetPose);
};

export const queryWorldPose = (model, ref) => {
  const referenceFeature = model[ref];
  let position = referenceFeature.getWorldPosition(new Vector3());
  let rotation = referenceFeature.getWorldQuaternion(new Quaternion());
  return {
    position: {
      x: position.x,
      y: position.y,
      z: position.z,
    },
    rotation: {
      x: rotation.x,
      y: rotation.y,
      z: rotation.z,
      w: rotation.w,
    },
  };
};

export const queryLocalPose = (model, ref, localTransform) => {
  const referenceFeature = model[ref];
  // create a temporary group to add to that reference;
  const worldPose = new Group();
  worldPose.position.set(
    localTransform.position.x,
    localTransform.position.y,
    localTransform.position.z
  );
  worldPose.quaternion.set(
    localTransform.rotation.x,
    localTransform.rotation.y,
    localTransform.rotation.z,
    localTransform.rotation.w
  );
  referenceFeature.attach(worldPose);
  const local = {
    position: {
      x: worldPose.position.x,
      y: worldPose.position.y,
      z: worldPose.position.z,
    },
    rotation: {
      x: worldPose.quaternion.x,
      y: worldPose.quaternion.y,
      z: worldPose.quaternion.z,
      w: worldPose.quaternion.w,
    },
  };
  worldPose.removeFromParent();
  return local;
};

export const createStaticEnvironment = (model) => {
    return Object.values(model).filter(item => item.userData.parent !== 'world' && item.userData.collisionInfo).map(item => {});
    // let retVal = [];
    // return Object.values(model).filter(item => item.userData.isCollisionObj).map(item => {
    //     // Convert position to world frame
    //     let {position, rotation} = queryWorldPose(model, item.uuid);

    //     let partialObject = {
    //         name: item.uuid,
    //         frame: 'world',
    //         physical: item.userData.physical,
    //         localTransform: {
    //             translation: [position.x, position.y, position.z],
    //             rotation: [rotation.x, rotation.y, rotation.z, rotation.w]
    //         }
    //     };

    //     // Create appropriate object
    //     if (item.userData.collisionType === "cube") {
    //         retVal.push(merge(partialObject, {
    //             type: 'Box',
    //             x: item.scale.x,
    //             y: item.scale.y,
    //             z: item.scale.z,
    //         }))
    //     } else if (item.userData.collisionType === "sphere") {
    //         retVal.push(merge(partialObject, {
    //             type:'Sphere',
    //             radius: item.userData.radius,
    //         }))
    //     } else if (["capsule", "cylinder"].includes(item.userData.collisionType)) {
    //         retVal.push(merge(partialObject, {
    //             type: item.userData.collisionType === "capsule" ? 'Capsule' : 'Cylinder',
    //             length: item.userData.length,
    //             radius: item.userData.radius,
    //         }))
    //     } 
    // }).filter(item => item !== null);
    // return retVal;
}

export const createEnvironmentModel = (programData) => {
  let added = true;
  let model = {};
  model.world = new Group();

  const handleItem = (item) => {
    const parentId = item.properties.relativeTo
      ? item.properties.relativeTo
      : "world";
    if (model[parentId]) {
      model[item.id] = new Group();
      model[item.id].userData.parent = parentId;

      // Only create collisions for non-link objects, as they are handled in the URDF
      if (false && item.properties.collision && item.type !== "linkType") {
        let collisionObjects =
          programData[item.properties.collision].properties.componentShapes;
        collisionObjects.forEach((collisionObjKey) => {
          model[collisionObjKey] = new Group();
          model[collisionObjKey].userData.parent = item.id;
          let collisionObj = programData[collisionObjKey];
          model[collisionObjKey].position.set(
            collisionObj.properties.position.x,
            collisionObj.properties.position.y,
            collisionObj.properties.position.z
          );
          model[collisionObjKey].quaternion.set(
            collisionObj.properties.rotation.x,
            collisionObj.properties.rotation.y,
            collisionObj.properties.rotation.z,
            collisionObj.properties.rotation.w
          );
          if (collisionObj.properties.keyword === "cube") {
            model[collisionObjKey].scale.set(
              collisionObj.properties.scale.x,
              collisionObj.properties.scale.y,
              collisionObj.properties.scale.z
            );
          } else if (collisionObj.properties.keyword === "sphere") {
            model[collisionObjKey].userData.radius =
              collisionObj.properties.radius;
          } else if (
            ["capsule", "cylinder"].includes(collisionObj.properties.keyword)
          ) {
            model[collisionObjKey].userData.radius =
              collisionObj.properties.radius;
            model[collisionObjKey].userData.length =
              collisionObj.properties.length;
          }

          model[collisionObjKey].userData.collisionType =
            collisionObj.properties.keyword;
          model[collisionObjKey].userData.isCollisionObj = true;
          model[collisionObjKey].userData.physical = !(
            item.type === "zoneType"
          );
          model[collisionObjKey].uuid = collisionObjKey;
          model[item.id].add(model[collisionObjKey]);
        });
      }
      model[item.id].position.set(
        item.properties.position.x,
        item.properties.position.y,
        item.properties.position.z
      );
      model[item.id].quaternion.set(
        item.properties.rotation.x,
        item.properties.rotation.y,
        item.properties.rotation.z,
        item.properties.rotation.w
      );
      model[item.id].uuid = item.id;
      model[parentId].add(model[item.id]);
      added = true;
      if (item.properties.gripOffset) {
        model[item.id + "-gripOffset"] = new Group();
        model[item.id + "-gripOffset"].userData.parent = item.id;
        model[item.id + "-gripOffset"].position.set(
          item.properties.gripPositionOffset.x,
          item.properties.gripPositionOffset.y,
          item.properties.gripPositionOffset.z
        );
        model[item.id + "-gripOffset"].quaternion.set(
          item.properties.gripRotationOffset.x,
          item.properties.gripRotationOffset.y,
          item.properties.gripRotationOffset.z,
          item.properties.gripRotationOffset.w
        );
        // For now, I am assuming no rotation
        model[item.id].add(model[item.id + "-gripOffset"]);
      }
    }
  };

  while (added) {
    added = false;
    Object.values(programData)
      .filter(
        (i) =>
          !Object.keys(model).includes(i.id) &&
          i.dataType === DATA_TYPES.INSTANCE &&
          REFERENCEABLE_OBJECTS.includes(i.type)
      )
      .forEach(handleItem);
  }
  return model;
};

export const anyReachable = (pose) => {
  let agents = Object.values(pose.properties.reachability);
  let reachable = false;
  agents.forEach(agent => {
      Object.values(agent).forEach(entry => {
          reachable = reachable || entry;
      });
  });
  return reachable;
}

export const checkHandThresholds = (pinchDistance) => {
  return (
    pinchDistance >= HAND_PINCH_MIN_DISTANCE &&
    pinchDistance <= HAND_PINCH_MAX_DISTANCE
  );
};

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
  // console.log({frames,model,frame})
  const linkTransforms = lodash.mapValues(frames, (frameData) => {
    const poseWorld = {
      position: {
        x: frameData.translation[0],
        y: frameData.translation[1],
        z: frameData.translation[2],
      },
      rotation: {
        w: frameData.rotation[3],
        x: frameData.rotation[0],
        y: frameData.rotation[1],
        z: frameData.rotation[2],
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

export const poseToGoalPosition = (model, gripperId, attachmentLink, pose) => {
  // console.log({model,gripperId,attachmentLink,pose})
  const poseOffset = computeRelativePose(
    model,
    gripperId + "-gripOffset",
    attachmentLink
  );
  const offsetMatrix = new Matrix4();
  const poseMatrix = new Matrix4();
  const positionVector = new Vector3(
    poseOffset.position.x,
    poseOffset.position.y,
    poseOffset.position.z
  );
  const rotationQuat = new Quaternion(
    poseOffset.rotation.x,
    poseOffset.rotation.y,
    poseOffset.rotation.z,
    poseOffset.rotation.w
  );
  // const positionVector = new Vector3(0,0,0.15);
  // const rotationQuat = new Quaternion(0.5,0.5,-0.5,0.5);
  const defaultScale = new Vector3(1, 1, 1);
  const goalPositionVector = new Vector3(
    pose.position.x,
    pose.position.y,
    pose.position.z
  );
  const goalRotationQuat = new Quaternion(
    pose.rotation.x,
    pose.rotation.y,
    pose.rotation.z,
    pose.rotation.w
  );
  offsetMatrix.compose(positionVector, rotationQuat, defaultScale);
  poseMatrix.compose(goalPositionVector, goalRotationQuat, defaultScale);
  poseMatrix.multiply(offsetMatrix);
  goalPositionVector.setFromMatrixPosition(poseMatrix);
  goalRotationQuat.setFromRotationMatrix(poseMatrix);
  return {
    position: {
      x: goalPositionVector.x,
      y: goalPositionVector.y,
      z: goalPositionVector.z,
    },
    rotation: {
      x: goalRotationQuat.x,
      y: goalRotationQuat.y,
      z: goalRotationQuat.z,
      w: goalRotationQuat.w,
    },
  };
};

export const typeToKey = (type) => {
  let key;
  switch (type) {
    case "trajectory":
      key = "trajectories";
      break;
    case "collisionMesh":
      key = "collisionMeshes";
      break;
    default:
      key = type + "s";
  }
  return key;
};

// function* range(start, end, step = 1) {
//   for (let i = start; i <= end; i += step) {
//     yield i;
//   }
// }

export function objectMap(object, mapFn) {
  return Object.keys(object).reduce(function (result, key) {
    result[key] = mapFn(object[key], key);
    return result;
  }, {});
}

export function arrayMove(arr, old_index, new_index) {
  while (old_index < 0) {
    old_index += arr.length;
  }
  while (new_index < 0) {
    new_index += arr.length;
  }
  if (new_index >= arr.length) {
    var k = new_index - arr.length + 1;
    while (k--) {
      arr.push(undefined);
    }
  }
  arr.splice(new_index, 0, arr.splice(old_index, 1)[0]);
  return arr; // for testing purposes
}

export function itemTransformMethod(state, id) {
  let idIncluded = false;
  let transformMethod = "inactive";

  // This variable determines whether the encountered translate/rotate applies to the item being searched
  let isTransformActive = false;

  state.focus.some((f) => {
    let focusItemType = state.programData[f]?.type;
    let focusTypeInfo = focusItemType
      ? Object.keys(state.programSpec.objectTypes[focusItemType].properties)
      : null;
    // Reset the isTransformActive, as the next translate/rotate would reference this new object
    if (focusTypeInfo && idIncluded && focusTypeInfo.includes("position")) {
      isTransformActive = false;
    }

    // Item exists in the focus array
    if (f === id) {
      idIncluded = true;
      isTransformActive = true;
    }

    // If item exists, and still has the potential focus of the translate/rotate, determine if translation/rotation applies
    if (idIncluded && isTransformActive && f === "translate") {
      transformMethod = "translate";
      return true;
    }
    if (idIncluded && isTransformActive && f === "rotate") {
      transformMethod = "rotate";
      return true;
    }
    return false;
  });

  return transformMethod;
}

export function deleteAction(data, uuid) {
  if (data[uuid].children) {
    /* Delete as a hierarchical */
    let children = data[uuid].children;
    let updated = lodash.omit(data, uuid);
    children.forEach((child) => {
      updated = deleteAction(data, child);
    });
    return updated;
  } else {
    return lodash.omit(data, uuid);
  }
}

export const occupancyOverlap = (position, occupancyZones) => {
  let overlap = false;
  let zones = Object.values(occupancyZones).filter(
    (v) => v.type === "zoneType"
  );
  for (let i = 0; i < zones.length; i++) {
    //.forEach(zone => {
    let zone = zones[i];
    const xOverlap =
      position.x < zone.properties.position.x + zone.properties.scale.x / 2 &&
      position.x > zone.properties.position.x - zone.properties.scale.x / 2;
    const yOverlap =
      position.y < zone.properties.position.z + zone.properties.scale.z / 2 &&
      position.y > zone.properties.position.z - zone.properties.scale.z / 2;
    if (xOverlap && yOverlap) {
      overlap = true;
    }
  }
  return overlap;
};

export function flattenProgram(primitives, skills, parentData) {
  let flattenedPrimitives = [];
  let flattenedSkills = [];

  primitives.forEach((primitive) => {
    if (primitive.type.includes("hierarchical")) {
      let newPrimitive = lodash.omit(primitive, "primitives");
      newPrimitive.children = primitive.primitives.map(
        (primitive) => primitive.uuid
      );
      newPrimitive.parentData = { type: "primitive", uuid: primitive };
      flattenedPrimitives.push(newPrimitive);
      const primitiveChildren = flattenProgram(primitive.primitives, [], {
        type: "primitive",
        uuid: primitive.uuid,
      })[0];
      flattenedPrimitives = [...flattenedPrimitives, ...primitiveChildren];
    } else {
      flattenedPrimitives.push({ ...primitive, parentData });
    }
  });
  skills.forEach((skill) => {
    if (skill.type.includes("hierarchical")) {
      let newSkill = lodash.omit(skill, "primitives");
      newSkill.children = skill.primitives.map((primitive) => primitive.uuid);
      flattenedSkills.push(newSkill);
      const primitiveChildren = flattenProgram(skill.primitives, [], {
        type: "skill",
        uuid: skill.uuid,
      })[0];
      flattenedPrimitives = [...flattenedPrimitives, ...primitiveChildren];
    }
  });

  return [flattenedPrimitives, flattenedSkills];
}

export const findLastSatisfiedFromReference = (x, fn) => {
  let lastIdx = x.length - 1;
  x.some((v, i) => {
    if (fn(v)) {
      lastIdx = i;
      return false;
    } else {
      return true;
    }
  });
  return lastIdx;
};

const pinchColorFromMagnitude = (magnitude = 0) => {
  return {
    r: 204 + 29 * magnitude,
    g: 121 - 68 * magnitude,
    b: 167 - 15 * magnitude,
    a: 0.3,
  };
};

const centerPoint = (point1, point2) => {
  return [
    (point1[0] + point2[0]) / 2,
    (point1[1] + point2[1]) / 2,
    (point1[2] + point2[2]) / 2,
  ];
};

const pinchPointVisualsByStep = (pairedLinks, proximity, previousDistances) => {
  let pinchPoints = {};

  if (!proximity) {
    return pinchPoints;
  }

  pairedLinks.forEach((pair) => {
    let link1 = pair["link1"];
    let link2 = pair["link2"];

    if (!pinchPoints[link1]) {
      pinchPoints[link1] = {};
    }

    if (
      proximity[link1] &&
      proximity[link1][link2] &&
      checkHandThresholds(proximity[link1][link2].distance) &&
      previousDistances[link1] &&
      previousDistances[link1][link2] &&
      previousDistances[link1][link2].distance -
        proximity[link1][link2].distance >
        0
    ) {
      let errorMagnitude =
        1 / Math.pow(Math.E, proximity[link1][link2].distance);
      let errorPosition =
        proximity[link1][link2]?.points?.length > 0
          ? centerPoint(
              proximity[link1][link2].points[0],
              proximity[link1][link2].points[1]
            )
          : [0, 0, 0];

      pinchPoints[link1][link2] = {
        scale: {
          x: errorMagnitude * 0.1,
          y: errorMagnitude * 0.1,
          z: errorMagnitude * 0.1,
        },
        color: pinchColorFromMagnitude(errorMagnitude),
        position: {
          x: errorPosition[0],
          y: errorPosition[1],
          z: errorPosition[2],
        },
      };
    } else {
      pinchPoints[link1][link2] = {
        scale: { x: 0, y: 0, z: 0 },
        color: { r: 0, g: 0, b: 0, a: 0 },
        position: { x: 0, y: 0, z: 0 },
      };
    }
  });
  return pinchPoints;
};

const stepsToAnimatedPinchPoints = (steps) => {
  if (steps.length === 0) {
    return {};
  }

  let tempAnimatedPinchPoints = {};
  let structure = {};

  Object.keys(steps[0].pinchPoints).forEach((link1) => {
    if (!structure[link1]) {
      structure[link1] = {};
    }

    Object.keys(steps[0].pinchPoints[link1]).forEach((link2) => {
      structure[link1][link2] = 0;
      tempAnimatedPinchPoints[link1 + "___" + link2] = {
        position: { x: [], y: [], z: [] },
        scale: { x: [], y: [], z: [] },
        color: { r: [], g: [], b: [] },
      };
    });
  });

  console.log(tempAnimatedPinchPoints);

  let timesteps = steps.map((step) => step.time);
  steps.forEach((step) => {
    Object.keys(step.pinchPoints).forEach((link1) => {
      Object.keys(step.pinchPoints[link1]).forEach((link2) => {
        tempAnimatedPinchPoints[link1 + "___" + link2].position.x.push(
          step.pinchPoints[link1][link2].position.x
        );
        tempAnimatedPinchPoints[link1 + "___" + link2].position.y.push(
          step.pinchPoints[link1][link2].position.y
        );
        tempAnimatedPinchPoints[link1 + "___" + link2].position.z.push(
          step.pinchPoints[link1][link2].position.z
        );
        tempAnimatedPinchPoints[link1 + "___" + link2].scale.x.push(
          step.pinchPoints[link1][link2].scale.x
        );
        tempAnimatedPinchPoints[link1 + "___" + link2].scale.y.push(
          step.pinchPoints[link1][link2].scale.y
        );
        tempAnimatedPinchPoints[link1 + "___" + link2].scale.z.push(
          step.pinchPoints[link1][link2].scale.z
        );
        tempAnimatedPinchPoints[link1 + "___" + link2].color.r.push(
          step.pinchPoints[link1][link2].color.r
        );
        tempAnimatedPinchPoints[link1 + "___" + link2].color.g.push(
          step.pinchPoints[link1][link2].color.g
        );
        tempAnimatedPinchPoints[link1 + "___" + link2].color.b.push(
          step.pinchPoints[link1][link2].color.b
        );
      });
    });
  });

  const animatedPinchPoints = objectMap(
    tempAnimatedPinchPoints,
    (pinchPoint) => ({
      frame: "world",
      position: {
        x: interpolateScalar(timesteps, pinchPoint.position.x),
        y: interpolateScalar(timesteps, pinchPoint.position.y),
        z: interpolateScalar(timesteps, pinchPoint.position.z),
      },
      scale: {
        x: interpolateScalar(timesteps, pinchPoint.scale.x),
        y: interpolateScalar(timesteps, pinchPoint.scale.y),
        z: interpolateScalar(timesteps, pinchPoint.scale.z),
      },
      color: {
        r: interpolateScalar(timesteps, pinchPoint.color.r),
        g: interpolateScalar(timesteps, pinchPoint.color.g),
        b: interpolateScalar(timesteps, pinchPoint.color.b),
        a: 0.3,
      },
    })
  );

  console.log(animatedPinchPoints);
  return animatedPinchPoints;
};

export function stepsToAnimation(state, tfs) {
  let dict = {};
  let lastTimestamp = {};
  let finalTime = 0;
  let timesteps = [];

  let focusStub = null;
  // Only back up to once for the animation correlating to the issue (if applicable)
  if (!state.programData[state.activeFocus]) {
    focusStub =
      state.programData[state.focus[state.focus.length - 2]]?.properties
        ?.compiled;
  } else {
    focusStub = state.programData[state.activeFocus]?.properties?.compiled;
  }
  const compileKeys = Object.keys(focusStub ? focusStub : {});

  // If too many or too few keys, return
  if (compileKeys.length !== 1) {
    return;
  }

  const compiledKey = compileKeys[0];

  // Build up the movements
  focusStub[compiledKey]?.steps?.forEach((step) => {
    if (step.type === STEP_TYPE.SCENE_UPDATE) {
      if (step.time > finalTime) {
        finalTime = step.time;
      }

      timesteps.push(step.time);

      // Add link rotation/position
      Object.keys(step.data.links).forEach((link) => {
        // Link didn't previously exist, so add it
        if (!dict[link]) {
          dict[link] = {
            position: { x: [], y: [], z: [] },
            rotation: { x: [], y: [], z: [], w: [] },
          };
          lastTimestamp[link] = 0;
        }
        // If just encountering link (after t iterations) use first data piece to backfill information
        if (
          lastTimestamp[link] === 0 &&
          lastTimestamp[link] + 1 !== timesteps.length
        ) {
          for (let i = 0; i < timesteps.length; i++) {
            dict[link].position.x.push(step.data.links[link].position.x);
            dict[link].position.y.push(step.data.links[link].position.y);
            dict[link].position.z.push(step.data.links[link].position.z);
            dict[link].rotation.x.push(step.data.links[link].rotation.x);
            dict[link].rotation.y.push(step.data.links[link].rotation.y);
            dict[link].rotation.z.push(step.data.links[link].rotation.z);
            dict[link].rotation.w.push(step.data.links[link].rotation.w);
          }
          // Use the old data to fill static position until current time
        } else if (
          lastTimestamp[link] !== 0 &&
          lastTimestamp[link] + 1 !== timesteps.length
        ) {
          let posLength = dict[link].position.x.length;
    
          for (let tmpLength = posLength; tmpLength < timesteps.length; tmpLength++) {
            dict[link].position.x.push(dict[link].position.x[posLength - 1]);
            dict[link].position.y.push(dict[link].position.y[posLength - 1]);
            dict[link].position.z.push(dict[link].position.z[posLength - 1]);
            dict[link].rotation.x.push(dict[link].rotation.x[posLength - 1]);
            dict[link].rotation.y.push(dict[link].rotation.y[posLength - 1]);
            dict[link].rotation.z.push(dict[link].rotation.z[posLength - 1]);
            dict[link].rotation.w.push(dict[link].rotation.w[posLength - 1]);
          }
        }

        // Add new time
        lastTimestamp[link] = timesteps.length;
        dict[link].position.x.push(step.data.links[link].position.x);
        dict[link].position.y.push(step.data.links[link].position.y);
        dict[link].position.z.push(step.data.links[link].position.z);
        dict[link].rotation.x.push(step.data.links[link].rotation.x);
        dict[link].rotation.y.push(step.data.links[link].rotation.y);
        dict[link].rotation.z.push(step.data.links[link].rotation.z);
        dict[link].rotation.w.push(step.data.links[link].rotation.w);
      });
    } else if (timesteps.length > 0) {
      timesteps.push(step.time);
      if (step.time > finalTime) {
        finalTime = step.time;
      }

      Object.keys(dict).forEach((link) => {
        let posLength = dict[link].position.x.length;
    
        for (let tmpLength = posLength; tmpLength < timesteps.length; tmpLength++) {
          dict[link].position.x.push(dict[link].position.x[posLength - 1]);
          dict[link].position.y.push(dict[link].position.y[posLength - 1]);
          dict[link].position.z.push(dict[link].position.z[posLength - 1]);
          dict[link].rotation.x.push(dict[link].rotation.x[posLength - 1]);
          dict[link].rotation.y.push(dict[link].rotation.y[posLength - 1]);
          dict[link].rotation.z.push(dict[link].rotation.z[posLength - 1]);
          dict[link].rotation.w.push(dict[link].rotation.w[posLength - 1]);
        }

        lastTimestamp[link] = timesteps.length;
      });
    }
  });

  // Iterate through final objects and buffer out times (add additional 500ms)
  // Animate
  finalTime += 500;
  timesteps.push(finalTime);
  Object.keys(dict).forEach((link) => {
    let posLength = dict[link].position.x.length;

    for (let tmpLength = posLength; tmpLength < timesteps.length; tmpLength++) {
      dict[link].position.x.push(dict[link].position.x[posLength - 1]);
      dict[link].position.y.push(dict[link].position.y[posLength - 1]);
      dict[link].position.z.push(dict[link].position.z[posLength - 1]);
      dict[link].rotation.x.push(dict[link].rotation.x[posLength - 1]);
      dict[link].rotation.y.push(dict[link].rotation.y[posLength - 1]);
      dict[link].rotation.z.push(dict[link].rotation.z[posLength - 1]);
      dict[link].rotation.w.push(dict[link].rotation.w[posLength - 1]);
    }

    if (tfs[link]) {
      tfs[link].position = {
        x: interpolateScalar(timesteps, dict[link].position.x),
        y: interpolateScalar(timesteps, dict[link].position.y),
        z: interpolateScalar(timesteps, dict[link].position.z),
      };
      tfs[link].rotation = {
        x: interpolateScalar(timesteps, dict[link].rotation.x),
        y: interpolateScalar(timesteps, dict[link].rotation.y),
        z: interpolateScalar(timesteps, dict[link].rotation.z),
        w: interpolateScalar(timesteps, dict[link].rotation.w),
      };
    }
  });
}

export function pinchpointAnimationFromExecutable(robotAgent, stepData) {
  let pairedLinks = robotAgent.properties.pinchPointPairLinks;

  let steps = [{ time: 0, pinchPoints: {} }];
  pairedLinks.forEach((pair) => {
    if (!steps[0].pinchPoints[pair["link1"]]) {
      steps[0].pinchPoints[pair["link1"]] = {};
    }
    steps[0].pinchPoints[pair["link1"]][pair["link2"]] = {
      scale: { x: 0, y: 0, z: 0 },
      position: { x: 0, y: 0, z: 0 },
      color: { r: 0, g: 0, b: 0, a: 0 },
    };
  });

  let currentTime = 0;
  let prevStep = lodash.cloneDeep(steps[steps.length - 1]);
  let previousDistances = {};

  stepData.forEach((step) => {
    let formattedProxData = likProximityAdjustment(
      robotAgent ? robotAgent.properties.pinchPointPairLinks : [],
      step.data?.proximity,
      false
    );
    prevStep.pinchPoints = {
      ...prevStep.pinchPoints,
      ...pinchPointVisualsByStep(
        pairedLinks,
        formattedProxData,
        previousDistances
      ),
    };
    prevStep.time = step.time;
    steps.push(prevStep);
    previousDistances = lodash.cloneDeep(formattedProxData);
    prevStep = lodash.cloneDeep(steps[steps.length - 1]);
    currentTime = step.time;
  });
  steps.push({
    ...lodash.cloneDeep(steps[steps.length - 1]),
    time: currentTime + 500,
  });

  return stepsToAnimatedPinchPoints(steps);
}

function interpolateScalar(x, y) {
  //const defaultFn = (v) => 0;
  if (x.length <= 0) {
    return null;
  }
  const interp = (v) => {
    const val = v > x[x.length - 1] ? v % x[x.length - 1] : v;
    let lastIdx = 0;
    for (let i = 0; i < x.length; i++) {
      if (x[i] <= val) {
        lastIdx = i;
      } else {
        break;
      }
    }
    return y[lastIdx];
  };
  return interp;
}

export function unFlattenProgramPrimitives(primitives, ids) {
  let unFlattenedPrimitives = [];
  ids.forEach((id) => {
    let newPrimitive = lodash.omit(primitives[id], "children");
    if (newPrimitive.type.includes("hierarchical")) {
      newPrimitive.primitives = unFlattenProgramPrimitives(
        primitives,
        primitives[id].children
      );
    }
    unFlattenedPrimitives.push(newPrimitive);
  });
  return unFlattenedPrimitives;
}

export function unFlattenProgramSkills(skills, primitives) {
  let unflattenedSkillSet = Object.values(skills);
  let unFlattenedSkills = [];
  unflattenedSkillSet.forEach((skill) => {
    let newSkill = lodash.omit(skill, "children");
    newSkill.primitives = unFlattenProgramPrimitives(
      primitives,
      skill.children
    );
    unFlattenedSkills.push(newSkill);
  });
  return unFlattenedSkills;
}

export function poseToColor(pose, frame, focused, occupancyZones) {
  let color = { r: 255, g: 255, b: 255, a: focused ? 1 : 0 };
  let pos = pose.refData
    ? pose.refData.properties.position
    : pose.properties.position;
  if (frame === "safety" && occupancyOverlap(pos, occupancyZones)) {
    color.r = 233;
    color.g = 53;
    color.b = 152;
  } else if (frame === "performance" && !pose.reachable) {
    color.r = 204;
    color.g = 75;
    color.b = 10;
  }
  return color;
}

export function poseDataToShapes(pose, frame, occupancyZones) {
  let pose_stored = pose;
  return [
    {
      uuid: `${pose_stored.id}-tag`,
      frame: "world",
      name: pose.name,
      shape: pose_stored.type.includes("location") ? "flag" : "tag",
      position: pose_stored.refData
        ? pose_stored.refData.properties.position
        : pose_stored.properties.position,
      rotation: { w: 1, x: 0, y: 0, z: 0 },
      scale: { x: -0.25, y: 0.25, z: 0.25 },
      highlighted: false,
      showName: false,
      color: poseToColor(pose_stored, frame, false, occupancyZones),
    },
    {
      uuid: `${pose_stored.id}-pointer`,
      frame: "world",
      shape: pose_stored.type.includes("location")
        ? "package://app/meshes/LocationMarker.stl"
        : "package://app/meshes/OpenWaypointMarker.stl",
      position: pose_stored.refData
        ? pose_stored.refData.properties.position
        : pose_stored.properties.position,
      rotation: pose_stored.refData
        ? pose_stored.refData.properties.rotation
        : pose_stored.properties.rotation,
      scale: { x: 1, y: 1, z: 1 },
      highlighted: false,
      showName: false,
      color: poseToColor(pose_stored, frame, false, occupancyZones),
    },
  ];
}

export function trajectoryDataToLine(
  trajectory,
  locations,
  waypoints,
  frame,
  reachableAndPerformance
) {
  // For the time being, enumerate the location and waypoints //197, 50, 154
  let points = [];
  if (trajectory.start_location_uuid) {
    let location = locations[trajectory.start_location_uuid];
    let position = {
      x: location.position.x,
      y: location.position.y,
      z: location.position.z,
    };

    if (
      (frame === "performance" && location.joints.reachable) ||
      frame === "quality" ||
      frame === "business"
    ) {
      points.push({ position, color: { r: 209, g: 0, b: 146, a: 1 } });
    } else {
      points.push({ position, color: poseToColor(location, frame, true) });
    }
  }
  trajectory.waypoint_uuids.forEach((waypoint_uuid) => {
    let waypoint = waypoints[waypoint_uuid];
    let position = {
      x: waypoint.position.x,
      y: waypoint.position.y,
      z: waypoint.position.z,
    };
    if (
      (frame === "performance" && waypoint.joints.reachable) ||
      frame === "quality" ||
      frame === "business"
    ) {
      points.push({ position, color: { r: 209, g: 0, b: 146, a: 1 } });
    } else {
      points.push({ position, color: poseToColor(waypoint, frame, true) });
    }
  });

  if (trajectory.end_location_uuid) {
    let location = locations[trajectory.end_location_uuid];
    let position = {
      x: location.position.x,
      y: location.position.y,
      z: location.position.z,
    };
    if (
      (frame === "performance" && location.joints.reachable) ||
      frame === "quality" ||
      frame === "business"
    ) {
      points.push({ position, color: { r: 209, g: 0, b: 146, a: 1 } });
    } else {
      points.push({ position, color: poseToColor(location, frame, true) });
    }
  }
  return {
    name: trajectory.name,
    frame: "world",
    width: 0,
    vertices: points,
  };
}

export const stepsToEEPoseScores = (frames, endPointFrames) => {
  let scores = [0];

  for (let i = 1; i < frames.length; i++) {
    const p1 = [frames[i].x, frames[i].y, frames[i].z];
    const p0 = [frames[i-1].x, frames[i-1].y, frames[i-1].z];
    const q1 = [endPointFrames[i].x, endPointFrames[i].y, endPointFrames[i].z];
    const movementVec = new Vector3(
      p1[0] - p0[0],
      p1[1] - p0[1],
      p1[2] - p0[2]
    );
    const directionVec = new Vector3(
      q1[0] - p1[0],
      q1[1] - p1[1],
      q1[2] - p1[2]
    );
    scores.push(
      (1000 * movementVec.manhattanLength()) /
        Math.pow(Math.E, 10 * movementVec.angleTo(directionVec))
    );
  }

  return scores;
}

export const stepsToVertices = (steps) => {
  let verts = [];
  steps.forEach(step => {
      Object.keys(step.data.links).forEach(link => {
          verts.push(new Vector3 (
            step.data.links[link].position.x,
            step.data.links[link].position.y,
            step.data.links[link].position.z
          ))
      })
  })
  return verts;
}

// export const traceToVertices = (trace) => {
//   let vertices = [];
//   ROBOT_FRAMES.forEach((frame) => {
//     for (let i = 0; i < trace.frames[frame].length; i += 10) {
//       vertices.push(new Vector3(...trace.frames[frame][i][0]));
//     }
//   });
//   return vertices;
// };

export const verticesToVolume = (vertices) => {
  if (vertices.length > 0) {
    let geometry = new ConvexGeometry(vertices);
    return getVolume(geometry);
  }
  
  return 0;
};

// export const spaceEstimate = (trace) => {
//   let geometry = new ConvexGeometry(traceToVertices(trace));
//   return getVolume(geometry);
// };

/*
https://discourse.threejs.org/t/volume-of-three-buffergeometry/5109
*/
function getVolume(geometry) {
  if (!geometry.isBufferGeometry) {
    console.log("'geometry' must be an indexed or non-indexed buffer geometry");
    return 0;
  }
  var isIndexed = geometry.index !== null;
  let position = geometry.attributes.position;
  let sum = 0;
  let p1 = new Vector3(),
    p2 = new Vector3(),
    p3 = new Vector3();
  if (!isIndexed) {
    let faces = position.count / 3;
    for (let i = 0; i < faces; i++) {
      p1.fromBufferAttribute(position, i * 3 + 0);
      p2.fromBufferAttribute(position, i * 3 + 1);
      p3.fromBufferAttribute(position, i * 3 + 2);
      sum += signedVolumeOfTriangle(p1, p2, p3);
    }
  } else {
    let index = geometry.index;
    let faces = index.count / 3;
    for (let i = 0; i < faces; i++) {
      p1.fromBufferAttribute(position, index.array[i * 3 + 0]);
      p2.fromBufferAttribute(position, index.array[i * 3 + 1]);
      p3.fromBufferAttribute(position, index.array[i * 3 + 2]);
      sum += signedVolumeOfTriangle(p1, p2, p3);
    }
  }
  return sum;
}

function signedVolumeOfTriangle(p1, p2, p3) {
  return p1.dot(p2.cross(p3)) / 6.0;
}

export function idleTimeEstimate(state, programSteps) {
  let delay = 0;
  let open = {};

  programSteps.forEach((step) => {
    // Add event to the open object, to track when it closes
    if (
      step.type === STEP_TYPE.ACTION_START &&
      ["delayType", "processWaitType"].includes(state[step.source].type)
    ) {
      open[step.source] = step.time;

      // Remove event from the open object and update the duration
    } else if (
      step.type === STEP_TYPE.ACTION_END &&
      ["delayType", "processWaitType"].includes(state[step.source].type)
    ) {
      delay += step.time - open[step.source];
      delete open[step.source];
    }
  });

  // Convert milliseconds to seconds
  return delay / 1000;
}

export function durationEstimate(programSteps) {
  // Return the time from the last element and convert from milliseconds to seconds
  if (programSteps.length === 0) {
    return 0;
  }
  return programSteps[programSteps.length - 1].time / 1000;
}

function distanceBetween(p1, p2) {
  return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
}

function isViewRect(entry) {
  return "top" in entry;
}

function cornersOfRectangle(
  rect,
  left = rect.offsetLeft,
  top = rect.offsetTop,
  transform
) {
  return [
    {
      x: left - transform.x / transform.zoom,
      y: top - transform.y / transform.zoom,
    },
    {
      x: left + (rect.width - transform.x) / transform.zoom,
      y: top - transform.y / transform.zoom,
    },
    {
      x: left - transform.x / transform.zoom,
      y: top + (rect.height - transform.y) / transform.zoom,
    },
    {
      x: left + (rect.width - transform.x) / transform.zoom,
      y: top + (rect.height - transform.y) / transform.zoom,
    },
  ];
}

export const thresholdedClosestCorners = ({
  collisionRect,
  droppableContainers,
  editorTransform,
}) => {
  //console.log(editorTransform)
  let minDistanceToCorners = Infinity;
  let minDistanceContainer = null;
  const corners = cornersOfRectangle(
    collisionRect,
    collisionRect.left,
    collisionRect.top,
    editorTransform
  );

  for (const droppableContainer of droppableContainers) {
    const {
      rect: { current: rect },
    } = droppableContainer;

    if (rect) {
      const rectCorners = cornersOfRectangle(
        rect,
        isViewRect(rect) ? rect.left : undefined,
        isViewRect(rect) ? rect.top : undefined,
        { x: 0, y: 0, zoom: 1 }
      );
      const distances = corners.reduce((accumulator, corner, index) => {
        return accumulator + distanceBetween(rectCorners[index], corner);
      }, 0);
      const effectiveDistance = Number((distances / 4).toFixed(4));

      if (effectiveDistance < minDistanceToCorners) {
        minDistanceToCorners = effectiveDistance;
        minDistanceContainer = droppableContainer.id;
      }
    }
  }
  return minDistanceContainer;
};
