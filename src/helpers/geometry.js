import { Quaternion, Vector3, Group, Object3D, Matrix4, Euler } from "three";
import { DATA_TYPES } from "simple-vp";
import { REFERENCEABLE_OBJECTS } from "../stores/Constants";
import { transformToThreeMatrix } from "./conversion";
import { strip } from "number-precision";
import { merge } from 'lodash';

Object3D.DefaultUp.set(0, 0, 1);

export const DEFAULT_SCALE = new Vector3(1, 1, 1);

export const getGoalTransformer = (positionOffset, rotationOffset, invert) => {
  let rotateValue = rotationOffset;
  if (typeof(rotateValue?.w) !== typeof(1)) {
    rotateValue = eulerToQuaternion(rotationOffset);
  }
  let m = new Matrix4().compose(
    new Vector3(positionOffset.x, positionOffset.y, positionOffset.z),
    new Quaternion(
      rotateValue.x,
      rotateValue.y,
      rotateValue.z,
      rotateValue.w
    ),
    new Vector3(1.0, 1.0, 1.0)
  );

  if (invert) {
    m = m.invert();
  }

  const transformer = (pose) => {
    let tmpRotate = pose.rotation;
    if (Array.isArray(pose.rotation)) {
      if (pose.rotation.length > 3) {
        tmpRotate = {x: pose.rotation[0], y: pose.rotation[1], z: pose.rotation[2], w: pose.rotation[3]}
      } else {
        tmpRotate = {x: pose.rotation[0], y: pose.rotation[1], z: pose.rotation[2]}
      }
    }

    if (typeof(tmpRotate?.w) !== typeof(1)) {
      tmpRotate = eulerToQuaternion(pose.rotation);
    }

    let originalM = new Matrix4().compose(
      new Vector3(
        pose.translation ? pose.translation[0] : pose.position.x,
        pose.translation ? pose.translation[1] : pose.position.y,
        pose.translation ? pose.translation[2] : pose.position.z
      ),
      new Quaternion(tmpRotate.x, tmpRotate.y, tmpRotate.z, tmpRotate.w),
      new Vector3(1.0, 1.0, 1.0)
    );

    let newM = new Matrix4().multiplyMatrices(originalM, m);
    let position = new Vector3().setFromMatrixPosition(newM);
    let rotation = new Quaternion().setFromRotationMatrix(newM);
    return {
      position: { x: position.x, y: position.y, z: position.z },
      rotation: { x: rotation.x, y: rotation.y, z: rotation.z, w: rotation.w },
    };
  };
  return transformer;
};

export const distance = (pos1, pos2) => {
  return Math.sqrt(
    Math.pow(pos1.x - pos2.x, 2) +
      Math.pow(pos1.y - pos2.y, 2) +
      Math.pow(pos1.z - pos2.z, 2)
  );
};

const _AXES2TUPLE = {
  sxyz: [0, 0, 0, 0],
  sxyx: [0, 0, 1, 0],
  sxzy: [0, 1, 0, 0],
  sxzx: [0, 1, 1, 0],
  syzx: [1, 0, 0, 0],
  syzy: [1, 0, 1, 0],
  syxz: [1, 1, 0, 0],
  syxy: [1, 1, 1, 0],
  szxy: [2, 0, 0, 0],
  szxz: [2, 0, 1, 0],
  szyx: [2, 1, 0, 0],
  szyz: [2, 1, 1, 0],
  rzyx: [0, 0, 0, 1],
  rxyx: [0, 0, 1, 1],
  ryzx: [0, 1, 0, 1],
  rxzx: [0, 1, 1, 1],
  rxzy: [1, 0, 0, 1],
  ryzy: [1, 0, 1, 1],
  rzxy: [1, 1, 0, 1],
  ryxy: [1, 1, 1, 1],
  ryxz: [2, 0, 0, 1],
  rzxz: [2, 0, 1, 1],
  rxyz: [2, 1, 0, 1],
  rzyz: [2, 1, 1, 1],
};

const _NEXT_AXIS = [1, 2, 0, 1];

const _EPS = 8.881784197001252 * Math.pow(10, -16);

const dot = (a, b) => a.map((x, i) => a[i] * b[i]).reduce((m, n) => m + n);

const outer = (a, b) => a.map((x) => b.map((y) => x * y));

export const quaternionVecToObject = (vec4) => ({
  w: strip(vec4[0]),
  x: strip(vec4[1]),
  y: strip(vec4[2]),
  z: strip(vec4[3]),
});

export const quaternionFromEuler = (vec3, axes = "szxy") => {
  /*
Return quaternion from Euler angles and axis sequence.
ai, aj, ak : Euler's roll, pitch and yaw angles
axes : One of 24 axis sequences as string or encoded tuple
>>> q = quaternion_from_euler(1, 2, 3, 'ryxz')
>>> numpy.allclose(q, [0.435953, 0.310622, -0.718287, 0.444435])
True
*/
  // let [ai, aj, ak] = [...vec3];
  // let [firstaxis, parity, repetition, frame] = [
  //   ..._AXES2TUPLE[axes.toLowerCase()],
  // ];

  // let i = firstaxis + 1;
  // let j = _NEXT_AXIS[i + parity - 1] + 1;
  // let k = _NEXT_AXIS[i - parity] + 1;

  // if (frame !== 0) {
  //   let values = [...[ai, ak]];
  //   ai = values[1];
  //   ak = values[0];
  // }
  // if (parity !== 0) {
  //   aj = -1 * aj;
  // }

  // ai = ai / 2.0;
  // aj = aj / 2.0;
  // ak = ak / 2.0;
  // let ci = Math.cos(ai);
  // let si = Math.sin(ai);
  // let cj = Math.cos(aj);
  // let sj = Math.sin(aj);
  // let ck = Math.cos(ak);
  // let sk = Math.sin(ak);
  // let cc = ci * ck;
  // let cs = ci * sk;
  // let sc = si * ck;
  // let ss = si * sk;

  // let q = [null, null, null, null];
  // if (repetition !== 0) {
  //   q[0] = cj * (cc - ss);
  //   q[i] = cj * (cs + sc);
  //   q[j] = sj * (cc + ss);
  //   q[k] = sj * (cs - sc);
  // } else {
  //   q[0] = cj * cc + sj * ss;
  //   q[i] = cj * sc - sj * cs;
  //   q[j] = cj * ss + sj * cc;
  //   q[k] = cj * cs - sj * sc;
  // }

  // if (parity !== 0) {
  //   q[j] *= -1.0;
  // }
  
  // return q;
  return new Quaternion().setFromEuler(new Euler(vec3.x, vec3.y, vec3.z));
};


const RAD_2_DEG = 180 / Math.PI;
const DEG_2_RAD = Math.PI / 180;

export const eulerToDegrees = (vec) => {
  return {x: vec.x * RAD_2_DEG, y: vec.y * RAD_2_DEG, z: vec.z * RAD_2_DEG};
};
export const eulerToRadians = (vec) => {
  return {x: vec.x * DEG_2_RAD, y: vec.y * DEG_2_RAD, z: vec.z * DEG_2_RAD};
};


export const eulerToQuaternion = (vec3, asThree) => {
  let radianEuler = eulerToRadians(vec3);
  let tmp = new Quaternion().setFromEuler(new Euler(radianEuler.x, radianEuler.y, radianEuler.z));
  if (asThree) return tmp;
  return {x: tmp.x, y: tmp.y, z: tmp.z, w: tmp.w};
}

export const quaternionToEuler = (quaternion) => {
  let tmp = eulerToDegrees(new Euler().setFromQuaternion(new Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)));
  return {x: tmp.x, y: tmp.y, z: tmp.z};
}

export const eulerFromMatrix = (matrix, axes = "szxy") => {
  /*
  Return Euler angles from rotation matrix for specified axis sequence.
  axes : One of 24 axis sequences as string or encoded tuple
  Note that many Euler angle triplets can describe one matrix.
  >>> R0 = euler_matrix(1, 2, 3, 'syxz')
  >>> al, be, ga = euler_from_matrix(R0, 'syxz')
  >>> R1 = euler_matrix(al, be, ga, 'syxz')
  >>> numpy.allclose(R0, R1)
  True
  >>> angles = (4*math.pi) * (numpy.random.random(3) - 0.5)
  >>> for axes in _AXES2TUPLE.keys():
  ...    R0 = euler_matrix(axes=axes, *angles)
  ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
  ...    if not numpy.allclose(R0, R1): print(axes, "failed")
  */
  let [firstaxis, parity, repetition, frame] = [
    ..._AXES2TUPLE[axes.toLowerCase()],
  ];

  let i = firstaxis;
  let j = _NEXT_AXIS[i + parity];
  let k = _NEXT_AXIS[i - parity + 1];

  let ax = 0.0;
  let ay = 0.0;
  let az = 0.0;

  let M = JSON.parse(JSON.stringify(matrix));

  if (repetition !== 0) {
    let sy = Math.sqrt(M[i][j] * M[i][j] + M[i][k] * M[i][k]);
    if (sy > _EPS) {
      ax = Math.atan2(M[i][j], M[i][k]);
      ay = Math.atan2(sy, M[i][i]);
      az = Math.atan2(M[j][i], -M[k][i]);
    } else {
      ax = Math.atan2(-M[j][k], M[j][j]);
      ay = Math.atan2(sy, M[i][i]);
      az = 0.0;
    }
  } else {
    let cy = Math.sqrt(M[i][i] * M[i][i] + M[j][i] * M[j][i]);
    if (cy > _EPS) {
      ax = Math.atan2(M[k][j], M[k][k]);
      ay = Math.atan2(-M[k][i], cy);
      az = Math.atan2(M[j][i], M[i][i]);
    } else {
      ax = Math.atan2(-M[j][k], M[j][j]);
      ay = Math.atan2(-M[k][i], cy);
      az = 0.0;
    }
  }

  if (parity !== 0) {
    [ax, ay, az] = [...[-ax, -ay, -az]];
  }
  if (frame !== 0) {
    [ax, az] = [...[az, ax]];
  }
  return [ax, ay, az];
};

export const quaternionMatrix = (quaternion) => {
  /*
  Return homogeneous rotation matrix from quaternion.
  >>> M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
  >>> numpy.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
  True
  >>> M = quaternion_matrix([1, 0, 0, 0])
  >>> numpy.allclose(M, numpy.identity(4))
  True
  >>> M = quaternion_matrix([0, 1, 0, 0])
  >>> numpy.allclose(M, numpy.diag([1, -1, -1, 1]))
  True
  */
  let q = [...quaternion];
  let n = dot(q, q);
  if (n < _EPS) {
    return [
      [1, 0, 0, 0],
      [0, 1, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1],
    ];
  }

  q = q.map((v) => v * Math.sqrt(2.0 / n));
  q = outer(q, q);
  return [
    [1.0 - q[2][2] - q[3][3], q[1][2] - q[3][0], q[1][3] + q[2][0], 0.0],
    [q[1][2] + q[3][0], 1.0 - q[1][1] - q[3][3], q[2][3] - q[1][0], 0.0],
    [q[1][3] - q[2][0], q[2][3] + q[1][0], 1.0 - q[1][1] - q[2][2], 0.0],
    [0.0, 0.0, 0.0, 1.0],
  ];
};

export const eulerFromQuaternion = (quaternion, axes = "szxy") => {
  return eulerFromMatrix(quaternionMatrix(quaternion), axes);
};

export const quaternionExp = (vec3) => {
  /*
  Returns the exponentiated quaternion from rotation vector vec3
  :param vec3:
  :return: quaternion in format [w x y z]
  */
  let q = [1.0, vec3[0], vec3[1], vec3[2]];
  let a = Math.sqrt(
    Math.pow(q[0], 2) +
      Math.pow(q[1], 2) +
      Math.pow(q[2], 2) +
      Math.pow(q[3], 2)
  );
  let sina = Math.sin(a);
  if (Math.abs(sina) >= 0.005) {
    let c = sina / a;
    q[1] *= c;
    q[2] *= c;
    q[3] *= c;
  }

  q[0] = Math.cos(a);

  return q;
};

export const quaternionSlerp = (
  quat0,
  quat1,
  fraction,
  spin = 0,
  shortestpath = true
) => {
  /*Return spherical linear interpolation between two quaternions.
>>> q0 = random_quaternion()
>>> q1 = random_quaternion()
>>> q = quaternion_slerp(q0, q1, 0)
>>> numpy.allclose(q, q0)
True
>>> q = quaternion_slerp(q0, q1, 1, 1)
>>> numpy.allclose(q, q1)
True
>>> q = quaternion_slerp(q0, q1, 0.5)
>>> angle = math.acos(numpy.dot(q0, q))
>>> numpy.allclose(2, math.acos(numpy.dot(q0, q1)) / angle) or \
    numpy.allclose(2, math.acos(-numpy.dot(q0, q1)) / angle)
True
*/
  let q0 = unitVector(quat0);
  let q1 = unitVector(quat1);
  if (fraction === 0.0) {
    return q0;
  } else if (fraction === 1.0) {
    return q1;
  }

  let d = dot(q0, q1);

  if (Math.abs(Math.abs(d) - 1.0) < _EPS) {
    return q0;
  }
  if (shortestpath && d < 0.0) {
    // invert rotation
    d = -d;
    q1 = q1.map((v) => -1 * v);
  }

  let angle = Math.acos(d) + spin * Math.PI;
  if (Math.abs(angle) < _EPS) {
    return q0;
  }

  let isin = 1.0 / Math.sin(angle);
  q0 = q0.map((v) => v * Math.sin((1.0 - fraction) * angle) * isin);
  q1 = q1.map((v) => v * Math.sin(fraction * angle) * isin);
  q0 = q0.map((v, i) => v * q1[i]);
  return q0;
};

const unitVector = (data) => {
  /*Return normalized by length, i.e. Euclidean norm, along axis.
   */
  let scale = Math.sqrt(dot(data, data));
  return data.map((v) => v * scale);
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
    let radians = eulerToRadians(transform.rotation);
    model[transformId].rotation.set(
      radians.x,
      radians.y,
      radians.z
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

export const getUserDataFromModel = (model, itemId, field) => {
  return model[itemId]?.userData[field];
}

export const getAllChildrenFromModel = (model, itemId) => {
  if (model[itemId]) {
    let ids = [];
    model[itemId].children.forEach(group => {
      ids.push(group.uuid);
    })
    return ids;
  }
  return null;
}

export const addGraspPointToModel = (model, parentId, itemId, position, rotation, width) => {
  if (model[parentId]) {
    model[itemId] = new Group();
    model[itemId].userData.parent = parentId;

    model[itemId].position.set(
      position.x,
      position.y,
      position.z
    );
    let radians = eulerToRadians(rotation);
    model[itemId].rotation.set(
      radians.x,
      radians.y,
      radians.z
    );
    model[itemId].uuid = itemId;
    model[itemId].userData['width'] = width;
    model[parentId].add(model[itemId]);
  }
  return model;
}

export const addToEnvironModel = (model, parentId, itemId, position, rotation) => {
  if (model[parentId]) {
    if (model[itemId]) {} else {
      model[itemId] = new Group();
    }
    model[itemId].userData.parent = parentId;

    model[itemId].position.set(
      position.x,
      position.y,
      position.z
    );
    model[itemId].quaternion.set(
      rotation.x,
      rotation.y,
      rotation.z,
      rotation.w
    );
    model[itemId].uuid = itemId;
    model[parentId].add(model[itemId]);
  }
  return model;
}

export const updateEnvironModel = (model, itemId, position, rotation) => {
  if (model[itemId]) {
    model[itemId].position.set(
      position.x,
      position.y,
      position.z
    );
    model[itemId].quaternion.set(0,0,0,1);
    let radians = eulerToRadians(rotation);
    model[itemId].rotation.set(
      radians.x,
      radians.y,
      radians.z
    );
  }
  return model;
}

export const updateEnvironModelQuaternion = (model, itemId, position, rotation) => {
  if (model[itemId]) {
    model[itemId].position.set(
      position.x,
      position.y,
      position.z
    );
    model[itemId].rotation.set(0,0,0);
    model[itemId].quaternion.set(
      rotation.x,
      rotation.y,
      rotation.z,
      rotation.w
    );
  }
  return model;
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
      if (item.properties.collision && item.type !== "linkType" && item.type !== "toolType" && item.type !== "thingType") {
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
          let radians = eulerToRadians(collisionObj.properties.rotation);
          model[collisionObjKey].rotation.set(
            radians.x,
            radians.y,
            radians.z
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
      let itemRadians = eulerToRadians(item.properties.rotation);
      model[item.id].rotation.set(
        itemRadians.x,
        itemRadians.y,
        itemRadians.z
      );
      model[item.id].uuid = item.id;
      model[parentId].add(model[item.id]);
      added = true;
      if (item.properties.gripPositionOffset) {
        const gripKey = `${item.id}-gripOffset`;
        model[gripKey] = new Group();
        model[gripKey].userData.parent = item.id;
        model[gripKey].position.set(
          item.properties.gripPositionOffset.x,
          item.properties.gripPositionOffset.y,
          item.properties.gripPositionOffset.z
        );
        let gripRotation = eulerToRadians(item.properties.gripRotationOffset);
        model[gripKey].rotation.set(
          gripRotation.x,
          gripRotation.y,
          gripRotation.z
        );
        model[item.id].add(model[gripKey]);
      }
    }
  };

  let allObjectTypes = ['inputOutputType', 'zoneType'];
  REFERENCEABLE_OBJECTS.forEach(t => allObjectTypes.push(t));
  while (added) {
    added = false;
    Object.values(programData)
      .filter(
        (i) =>
          !Object.keys(model).includes(i.id) &&
          i.dataType === DATA_TYPES.INSTANCE &&
          allObjectTypes.includes(i.type)
      )
      .forEach(handleItem);
  }
  return model;
};

export const createStaticEnvironment = (model) => {
  let retVal = [];
  Object.values(model).filter(item => item.userData.isCollisionObj).forEach(item => {
      // Convert position to world frame
      let {position, rotation} = queryWorldPose(model, item.uuid);

      let partialObject = {
          name: item.uuid,
          frame: 'world',
          physical: item.userData.physical,
          localTransform: {
              translation: [position.x, position.y, position.z],
              rotation: [rotation.x, rotation.y, rotation.z, rotation.w]
          }
      };

      // Create appropriate object
      if (item.userData.collisionType === "cube") {
          retVal.push(merge(partialObject, {
              type: 'Box',
              x: item.scale.x,
              y: item.scale.y,
              z: item.scale.z,
          }));
      } else if (item.userData.collisionType === "sphere") {
          retVal.push(merge(partialObject, {
              type:'Sphere',
              radius: item.userData.radius,
          }));
      } else if (["capsule", "cylinder"].includes(item.userData.collisionType)) {
          retVal.push(merge(partialObject, {
              type: item.userData.collisionType === "capsule" ? 'Capsule' : 'Cylinder',
              length: item.userData.length,
              radius: item.userData.radius,
          }));
      }
  });
  return retVal;
};
