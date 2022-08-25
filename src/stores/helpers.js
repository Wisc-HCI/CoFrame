import lodash from "lodash";
// import { GRIPPER_CONFIGURATIONS, GRIPPER_FRAMES, GRIPPER_PARENTS } from './gripper';
import { Vector3 } from "three";
import { ConvexGeometry } from "three-stdlib";
// import { EVD_MESH_LOOKUP } from './initialSim';
// import { DATA_TYPES } from "simple-vp";
import {
  HAND_PINCH_MAX_DISTANCE,
  HAND_PINCH_MIN_DISTANCE,
  // REFERENCEABLE_OBJECTS,
  ROOT_PATH,
  STEP_TYPE,
} from "./Constants";
// import { merge } from "lodash";


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

export const DEFAULT_LOCATION_COLOR = { r: 62, g: 16, b: 102, a: 1 };

export const DEFAULT_WAYPOINT_COLOR = { r: 100, g: 18, b: 128, a: 1 };

export const DEFAULT_TRAJECTORY_COLOR = { r: 209, g: 0, b: 146, a: 1 };

export const UNREACHABLE_COLOR = { r: 204, g: 75, b: 10, a: 1 };

export const OCCUPANCY_ERROR_COLOR = { r: 233, g: 53, b: 152, a: 1 };

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

export function interpolateScalar(x, y) {
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

export function getIDsAndStepsFromCompiled(program, programData, stepType, blockType, compiledData) {
  let ids = []
  let steps =  compiledData[program.id]?.[ROOT_PATH]?.steps?.filter(v => v.type === stepType) || [];
  steps.forEach((step) => {
      if (step.source && programData[step.source].type === blockType && !ids.includes(step.source)) {
        ids.push(step.source);
      }
  });

  return [ids, steps];
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
