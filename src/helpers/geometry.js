import { Quaternion, Vector3, Group, Object3D, Matrix4 } from "three";
import { DATA_TYPES } from "simple-vp";
import { REFERENCEABLE_OBJECTS } from "../stores/Constants";
import { transformToThreeMatrix } from "./conversion";

Object3D.DefaultUp.set(0, 0, 1);

export const DEFAULT_SCALE = new Vector3(1, 1, 1);

export const distance = (pos1, pos2) => {
  return Math.sqrt(
    Math.pow(pos1.x - pos2.x, 2) +
      Math.pow(pos1.y - pos2.y, 2) +
      Math.pow(pos1.z - pos2.z, 2)
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

export const poseToGoalPosition = (
  model,
  gripperId,
  attachmentLink,
  pose,
  returnAs
) => {
  // Calculate the offset of the attachment link from the gripper offset
  const poseOffset = computeRelativePose(
    model,
    gripperId + "-gripOffset",
    attachmentLink
  );
  const offsetMatrix = transformToThreeMatrix(poseOffset);
  const poseMatrix =
    pose.position && pose.rotation ? transformToThreeMatrix(pose) : pose;
  poseMatrix.multiply(offsetMatrix);
  if (returnAs === "three") {
    return poseMatrix;
  } else {
    const goalPositionVector = new Vector3();
    const goalRotationQuat = new Quaternion();
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
  }
};

export const attachmentToEEPose = (
  model,
  gripperId,
  attachmentLink,
  pose,
  returnAs
) => {
  // Calculate the offset of the attachment link from the gripper offset
  const poseOffset = computeRelativePose(
    model,
    attachmentLink,
    gripperId + "-gripOffset"
  );
  const offsetMatrix = transformToThreeMatrix(poseOffset);
  const poseMatrix =
    pose.position && pose.rotation ? transformToThreeMatrix(pose) : pose;
  poseMatrix.multiply(offsetMatrix);
  if (returnAs === "three") {
    return poseMatrix;
  } else {
    const goalPositionVector = new Vector3();
    const goalRotationQuat = new Quaternion();
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
  }
};

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

export const queryWorldPose = (model, ref, returnAs) => {
  const referenceFeature = model[ref];
  let position = referenceFeature.getWorldPosition(new Vector3());
  let rotation = referenceFeature.getWorldQuaternion(new Quaternion());
  if (returnAs === "three") {
    let matrix = new Matrix4();
    matrix.compose(position, rotation, DEFAULT_SCALE);
    return matrix;
  } else {
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
  }
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
      if (item.properties.gripPositionOffset) {
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

export const createStaticEnvironment = (model) => {
  return Object.values(model)
    .filter(
      (item) => item.userData.parent !== "world" && item.userData.collisionInfo
    )
    .map((item) => {});
  // let retVal = [];
  // Object.values(model).filter(item => item.userData.isCollisionObj).forEach(item => {
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
  // })
  // return retVal;
};
