import { delayCompiler } from "./delay";
import { gripperMotionCompiler } from "./gripperMotion";
import { machineCompiler } from "./machine";
import { nullCompiler } from "./null";
import { poseCompiler } from "./pose";
import { processCompiler } from "./process";
import { robotMotionCompiler } from "./robotMotion";
import { simpleCompiler } from "./simple";
import { breakCompiler } from "./break";
import { robotAgentCompiler } from "./robotAgent";
import { humanAgentCompiler } from "./humanAgent";
import { linkCompiler } from "./link";
import { propertyCompiler } from "./property";
import lodash from "lodash";
import { COMPILE_FUNCTIONS, ERROR, ROOT_PATH, STATUS } from "../Constants";
import { DATA_TYPES } from "simple-vp";
import { gripperCompiler } from "./gripper";

const KEY_MAPPING = {
  NULL: nullCompiler,
  SIMPLE: simpleCompiler,
  MACHINE: machineCompiler,
  DELAY: delayCompiler,
  BREAK: breakCompiler,
  PROCESS: processCompiler,
  POSE: poseCompiler,
  ROBOT_MOTION: robotMotionCompiler,
  GRIPPER_MOTION: gripperMotionCompiler,
  ROBOT_AGENT: robotAgentCompiler,
  HUMAN_AGENT: humanAgentCompiler,
  GRIPPER: gripperCompiler,
  LINK: linkCompiler,
  PROPERTY: propertyCompiler,
};
// Ordering corresponds to the values in the COMPILE_FUNCTIONS constant

export const compilers = Object.keys(COMPILE_FUNCTIONS).map(
  (k) => KEY_MAPPING[k]
);

export const findInstance = (id, context) => {
  let found = false;
  let stale = false;
  let data = undefined;
  let dataId = id;

  while (!found && !stale) {
    data = context[dataId];
    if (!data) {
      stale = true;
    } else if (data.dataType === DATA_TYPES.INSTANCE) {
      found = true;
    } else if (data.dataType === DATA_TYPES.REFERENCE) {
      dataId = data.ref;
    } else {
      stale = true;
    }
  }

  return data;
};

export const equals = (a, b) => JSON.stringify(a) === JSON.stringify(b);

// Copies data from a memo into the specified node.
// Returns standard changes, which will need to be propagated back up.
const copyMemoizedData = (memoizedData, data, path, singleton) => {

  if (memoizedData.properties.compiled[path]) {
    return [
      memoizedData, // memoizedData
      memoizedData.properties.compiled[path].status, // status
      memoizedData.properties.compiled[path].errorCode, // errorCode
      data.properties ? data.properties.status === STATUS.PENDING : true, // updated
      memoizedData.properties.compiled[path].break, // shouldBreak
    ];
  } else if (singleton && memoizedData.properties.compiled[ROOT_PATH]) {
    return [
      memoizedData, // memoizedData
      memoizedData.properties.compiled[ROOT_PATH].status, // status
      memoizedData.properties.compiled[ROOT_PATH].errorCode, // errorCode
      data.properties ? data.properties.status === STATUS.PENDING : true, // updated
      memoizedData.properties.compiled[ROOT_PATH].break, // shouldBreak
    ];
  }
  
};

// Copies data into memoized data.
// Returns standard changes, which will need to be propagated back up.
const updateMemoizedData = (memoizedData, data, path, singleton) => {
  // console.log("updateMemoizedData", { memoizedData, path, data, singleton });
  // console.warn('updating memoized version of ', data);
  let newMemoizedData = lodash.merge(
    { properties: { compiled: { [path]: {} } } },
    memoizedData
  );
  let pastCompiled = data.properties.compiled[path];
  if (!pastCompiled && singleton && data.properties.compiled[ROOT_PATH]) {
    pastCompiled = data.properties.compiled[ROOT_PATH];
  }
  newMemoizedData.properties.compiled[path] = lodash.merge(
    newMemoizedData.properties.compiled[path],
    pastCompiled
  );
  if (newMemoizedData.properties.compiled[path].otherPropertyUpdates) {
    // console.log('Has other properties, updating...')
    newMemoizedData = lodash.merge(newMemoizedData, {
      properties:
        newMemoizedData.properties.compiled[path].otherPropertyUpdates,
    });
    // console.log('New with updates:', newMemoizedData)
  }
  return [
    newMemoizedData, // memoizedData
    pastCompiled.status, // status
    pastCompiled.errorCode, // errorCode
    data.properties ? data.properties.status === STATUS.PENDING : true, // updated
    pastCompiled.break, // shouldBreak
  ];
};

// Computes a new node, given that children have been updated.
// Returns standard changes, which will need to be propagated back up.
const performUpdate = (
  memoizedData,
  data,
  properties,
  routes,
  objectTypes,
  context,
  path,
  memo,
  module,
  worldModel,
  updateFn
) => {
  // console.warn('updating version of ', data);
  let newCompiled = updateFn({
    data,
    properties,
    routes,
    objectTypes,
    context,
    path,
    memo,
    module,
    worldModel,
  });
  // console.log('status',newCompiled.status)
  // console.log('memoizedData',memoizedData)
  // console.log('newCompiled',newCompiled)
  let newMemoizedData = lodash.merge(
    { properties: { compiled: { [path]: {} } } },
    memoizedData
  );
  newMemoizedData.properties.compiled[path] = lodash.merge(
    newMemoizedData.properties.compiled[path],
    newCompiled
  );
  if (newCompiled.otherPropertyUpdates) {
    // console.log('Has other properties, updating...')
    newMemoizedData = lodash.merge(newMemoizedData, {
      properties: newCompiled.otherPropertyUpdates,
    });
    // console.log('New with updates:', newMemoizedData)
  }
  // console.log(newMemoizedData)
  return [
    newMemoizedData, // memoizedData
    newCompiled.status, // status
    newCompiled.errorCode, // errorCode
    data.properties ? data.properties.status === STATUS.PENDING : true, // updated
    newCompiled.shouldBreak, // shouldBreak
  ];
};

const newContextAndPathFromCallFn = (context, path, call, fn) => {
  let newContext = {};
  let prevPath = JSON.parse(path);
  let valid = true;

  fn.arguments.forEach((argument) => {
    const inst = findInstance(call.properties[argument], context);
    newContext[argument] = inst;
    if (!inst) {
      valid = false;
    }
  });

  return [
    { ...context, ...newContext },
    JSON.stringify({
      ...prevPath,
      ...lodash.mapValues(newContext, (v) => (v ? v.id : null)),
    }),
    valid,
  ];
};

// Computes a property of a parent given the field data and field info.
// Can handle node-based and 'ignored' fields
// Returns standard changes, which will need to be propagated back up.
const computeProperty = (
  fieldValue,
  fieldInfo,
  objectTypes,
  context,
  path,
  memo,
  module,
  worldModel
) => {
  let memoizedData = {};
  let newMemo = { ...memo };
  let status = STATUS.VALID;
  let updated = false;
  let shouldBreak = false;
  let exportPath = path;

  // First, if the field value is null, return null, with checking for validity
  if (!fieldValue && !fieldInfo.nullValid) {
    status = STATUS.FAILED;
  }
  if (!fieldValue) {
    return { memoizedData, memo: newMemo, status, updated, shouldBreak };
  }

  if (fieldInfo.accepts) {
    const propData = findInstance(fieldValue, context);
    // Check cases where the result is null here
    if (!propData && fieldInfo.nullValid) {
      return { memoizedData, memo: newMemo, status, updated, shouldBreak };
    } else if (!propData) {
      status = STATUS.FAILED;
    } else {
      // Since the prop is valid, continue
      const computeProps = {
        data: propData,
        objectTypes,
        context,
        path,
        memo: newMemo,
        module,
        worldModel,
      };

      const {
        memoizedData: innerMemoizedData,
        memo: innerMemo,
        status: innerStatus,
        updated: innerUpdated,
        shouldBreak: innerShouldBreak,
        exportPath: innerExportPath,
      } = handleUpdate(computeProps);
      if (innerUpdated) {
        updated = true;
      }
      if (
        innerStatus === STATUS.FAILED ||
        (innerStatus === STATUS.WARN && status !== STATUS.FAILED)
      ) {
        // Status is failed/warned, so the parent is also.
        status = innerStatus;
      }
      if (innerShouldBreak) {
        shouldBreak = true;
      }
      exportPath = innerExportPath;
      memoizedData = innerMemoizedData;
      newMemo = lodash.merge(newMemo, innerMemo);
    }
  } else {
    memoizedData = fieldValue;
  }
  //   console.log('full output',{ memoizedData, memo: newMemo, status, updated, shouldBreak })
  return {
    memoizedData,
    memo: newMemo,
    status,
    updated,
    shouldBreak,
    exportPath,
  };
};

// Entry for recursive process of updating data.
// Returns standard changes, which will need to be propagated back up.
export const handleUpdate = ({
  data,
  objectTypes,
  context,
  path,
  memo,
  module,
  worldModel,
  force,
}) => {
  let newMemo = { ...memo };
  // console.warn('DATA:',{data,path,context,memo})
  const updateFn =
    compilers[objectTypes[data.type].properties.compileFn.default];
  let memoizedData = memo[data.id] ? memo[data.id] : {};
  let updated = data.properties.status === STATUS.PENDING || force;
  if (force) {
    // console.log("forcing update for ", data);
  }
  let status = STATUS.VALID;
  let shouldBreak = false;
  let exportPath = path;
  let errorCode = null;

  // First, determine whether we need to compute.
  // If the status is pending, recompute.
  // If not pending, check the `updateFields` for node-based updates.
  // If any of those have updates, recompute.

  // No change detected at this level, but we should progress though `updateFields` to be sure.
  let properties = {};
  let routes = {};
  if (data.dataType !== DATA_TYPES.CALL) {
    // HANDLE NON-CALL LOGIC (MOST THINGS) //

    if (memoizedData.properties?.compiled[path] || (memoizedData.properties?.singleton && memoizedData.properties?.compiled[ROOT_PATH])) {
      // Use the version in the memo
      [memoizedData, status, errorCode, updated, shouldBreak] = copyMemoizedData(
        memoizedData,
        data,
        path,
        data.properties?.singleton
      );
    } else {
      data.properties.updateFields.forEach((field) => {
        // console.log('checking field', field)
        if (objectTypes[data.type].properties[field].isList) {
          properties[field] = [];
          routes[field] = [];
          data.properties[field].some((fieldItem) => {
            const {
              memoizedData: innerMemoizedData,
              memo: innerMemo,
              status: innerStatus,
              updated: innerUpdated,
              shouldBreak: innerShouldBreak,
              exportPath: innerExportPath,
            } = computeProperty(
              fieldItem,
              objectTypes[data.type].properties[field],
              objectTypes,
              context,
              path,
              memo,
              module,
              worldModel
            );
            properties[field].push(innerMemoizedData);
            routes[field].push(innerExportPath);
            newMemo = lodash.merge(newMemo, innerMemo);
            if (innerUpdated) {
              updated = true;
            }
            if (
              innerStatus === STATUS.FAILED ||
              (innerStatus === STATUS.WARN && status !== STATUS.FAILED)
            ) {
              // Status is failed/warned, so the parent is also.
              status = innerStatus;
              errorCode = ERROR.CHILD_FAILED;
            }
            if (innerShouldBreak) {
              shouldBreak = true;
            }
            return innerShouldBreak;
          });
        } else {
          const {
            memoizedData: innerMemoizedData,
            memo: innerMemo,
            status: innerStatus,
            updated: innerUpdated,
            shouldBreak: innerShouldBreak,
            exportPath: innerExportPath,
          } = computeProperty(
            data.properties[field],
            objectTypes[data.type].properties[field],
            objectTypes,
            context,
            path,
            memo,
            module,
            worldModel
          );
          properties[field] = innerMemoizedData;
          routes[field] = innerExportPath;
          newMemo = lodash.merge(newMemo, innerMemo);
          if (innerUpdated) {
            updated = true;
          }
          if (
            innerStatus === STATUS.FAILED ||
            (innerStatus === STATUS.WARN && status !== STATUS.FAILED)
          ) {
            // Status is failed/warned, so the parent is also.
            status = innerStatus;
            errorCode = ERROR.CHILD_FAILED;
          }
          if (innerShouldBreak) {
            shouldBreak = true;
          }
        }
      });
      let newErrorCode = null;
      if (updated) {
        [memoizedData, status, newErrorCode, updated, shouldBreak] = performUpdate(
          memoizedData,
          data,
          properties,
          routes,
          objectTypes,
          context,
          path,
          newMemo,
          module,
          worldModel,
          updateFn
        );
        updated = true;
      } else {
        [memoizedData, status, newErrorCode, updated, shouldBreak] = updateMemoizedData(
          memoizedData,
          data,
          path,
          data.properties?.singleton
        );
      }
      if (newErrorCode) {
        errorCode = newErrorCode;
      }
    }
  } else {
    // HANDLE CALL LOGIC //
    // console.log("call found", data);
    const fnInstance = findInstance(data.ref, context);

    // Treat the function instance like it is a property, but swap in the new contexts.

    const [fnContext, fnPath] = newContextAndPathFromCallFn(
      context,
      path,
      data,
      fnInstance
    );
    exportPath = fnPath;
    let newErrorCode = null;

    if (memoizedData.properties?.compiled[exportPath] || (memoizedData.properties?.singleton && memoizedData.properties?.compiled[ROOT_PATH])) {
      // Use the version in the memo
      [memoizedData, status, newErrorCode, updated, shouldBreak] = copyMemoizedData(
        memoizedData,
        data,
        exportPath,
        memoizedData.properties?.singleton
      );
      if (newErrorCode) {
        errorCode = newErrorCode;
      }
    } else {
      fnInstance.arguments.forEach((argument) => {
        const argData = findInstance(argument, fnContext);
        // console.log({ argument, context, argData });
        if (!argData || argData.dataType === DATA_TYPES.ARGUMENT) {
          status = STATUS.FAILED;
          memoizedData = {
            properties: {
              compiled: {
                [exportPath]: {
                  shouldBreak: false,
                  status: STATUS.FAILED,
                  events: [],
                  steps: [],
                },
              },
              status: STATUS.FAILED,
            },
          };
          // console.log("failed arg check", { argument, context });
          // return { memoizedData, memo: newMemo, status, updated, shouldBreak, exportPath };
        }

        if (status !== STATUS.FAILED) {
          // console.log("passed arg check");
          const argType = argData.type;
          const fieldInfo = {
            accepts: [argType],
            isList: false,
            nullValid: false,
          };
          const {
            memoizedData: innerMemoizedData,
            memo: innerMemo,
            status: innerStatus,
            updated: innerUpdated,
            shouldBreak: innerShouldBreak,
          } = computeProperty(
            argument,
            fieldInfo,
            objectTypes,
            fnContext,
            exportPath,
            memo,
            module,
            worldModel
          );
          properties[argument] = innerMemoizedData;
          newMemo = lodash.merge(newMemo, innerMemo);
          if (innerUpdated) {
            updated = true;
          }
          if (
            innerStatus === STATUS.FAILED ||
            (innerStatus === STATUS.WARN && status !== STATUS.FAILED)
          ) {
            // Status is failed/warned, so the parent is also.
            status = innerStatus;
            errorCode = ERROR.CHILD_FAILED;
          }
          if (innerShouldBreak) {
            shouldBreak = true;
          }
        }
      });

      // console.log("Computing called instance");

      if (status !== STATUS.FAILED) {
        const {
          memoizedData: fnMemoizedData,
          memo: fnMemo,
          status: fnStatus,
          shouldBreak: fnShouldBreak,
          updated: fnUpdated,
        } = handleUpdate({
          data: fnInstance,
          objectTypes,
          context: fnContext,
          path: fnPath,
          memo: newMemo,
          module,
          worldModel,
          force: true,
        });
  
        memoizedData = { ...fnMemoizedData, id: data.id };
        newMemo = lodash.merge(newMemo, fnMemo);
        if (
          fnStatus === STATUS.FAILED ||
          (fnStatus === STATUS.WARN && status !== STATUS.FAILED)
        ) {
          // Status is failed/warned, so the parent is also.
          status = fnStatus;
          errorCode = ERROR.CHILD_FAILED;
        }
        if (fnShouldBreak) {
          shouldBreak = true;
        }
        if (fnUpdated) {
          updated = true;
        }
      }
      // console.log("fn memozizedData");
    }
  }

  memoizedData.properties.status = status;
  memoizedData.properties.errorCode = errorCode;
  if (memoizedData.properties.compiled[exportPath]) {
    memoizedData.properties.compiled[exportPath].break = shouldBreak;
  } else {
    console.warn("SHOULDBREAK ERROR: ", {
      memoizedData,
      exportPath,
    });
  }
  memoizedData.properties.compiled[exportPath].break = shouldBreak;
  memoizedData.type = data.type;
  memoizedData.id = data.id;

  newMemo = { ...newMemo, [data.id]: memoizedData };
  return {
    memoizedData,
    memo: newMemo,
    status,
    updated,
    shouldBreak,
    exportPath,
  };
};

const compileStates = (states, time) => {
  let newState = {};
  let cutoffTime = time ? time : Number.POSITIVE_INFINITY;
  states.some((state) => {
    if (state.time <= cutoffTime) {
      newState = lodash.merge(newState, state.current);
      return false;
    } else {
      return true;
    }
  });
  return newState;
};

const compileEffects = (steps) => {
  let effects = {};
  steps.forEach((step) => {
    effects = lodash.merge(effects, step.effect);
  });
  return effects;
};

const compare = (baseline, test) => {
  let matches = true;
  let matchedKeys = [];
  const allKeys = lodash.uniq([...Object.keys(baseline), ...Object.keys(test)]);
  allKeys.forEach((key) => {
    // console.log({key,baseline:baseline[key],test:test[key]})
    if (baseline[key] === undefined || test[key] === undefined) {
      // console.log('found one undefined',{matches,matchCount})
    } else if (
      typeof baseline[key] === "object" &&
      typeof test[key] === "object"
    ) {
      // console.log('both objects, inner compare...')
      const [innerMatches, innerMatchedKeys] = compare(
        baseline[key],
        test[key]
      );
      if (innerMatches) {
        matchedKeys.push(key);
      } else {
        matches = false;
      }
      matchedKeys = [
        ...matchedKeys,
        ...innerMatchedKeys.map((ik) => `${key}/${ik}`),
      ];
      // console.log('both objects, inner compare...',{matches,matchCount})
    } else if (
      typeof baseline[key] === "object" ||
      typeof test[key] === "object"
    ) {
      matches = false;
      // console.log('only one is object',{matches,matchCount})
    } else {
      const innerMatches = lodash.isEqual(baseline[key], test[key]);
      if (innerMatches) {
        matchedKeys.push(key);
      } else if (!innerMatches) {
        matches = false;
      }
      // console.log('neither is object',{key,matches,matchCount,innerMatches})
    }
  });

  return [matches, matchedKeys];
};

const withRefreshedStates = (states) => {
  // let currentState = {};
  states.sort((a, b) => a.time - b.time);
  // states.forEach((state,i)=>{
  //     currentState = lodash.merge(currentState,state.current);
  //     states[i].current = lodash.cloneDeep(currentState);
  // })
  return states;
};

const isSubset = (arr1, arr2) => {
  const intersection = lodash.intersection(arr1, arr2);
  return arr2.length === intersection.length;
};

const searchForTime = (states, condition, effects) => {
  let time = 0;
  let timeIdx = states.length;
  let found = states.length === 0;
  let savedMatches = [];
  states
    .slice()
    .reverse()
    .some((state, idx) => {
      // console.log('considering state match for:',{condition,state,effects})
      const prevState =
        idx === 0 ? { current: {} } : states[states.length - idx];
      // console.log('prevState',prevState)
      // console.log('state',state)
      const [currentMatch, currentMatches] = compare(state.current, condition);
      const [pastMatch, _] = compare(effects, prevState.current);
      // console.log({currentMatch, currentMatches, pastMatch, savedMatches})
      if (!pastMatch || !currentMatch) {
        // console.log('cancelling search');
        return true;
      } else if (
        currentMatch &&
        !isSubset(currentMatches, savedMatches) &&
        found
      ) {
        // console.log('past match superior, cancelling');
        return true;
      } else if (currentMatch) {
        // console.log('match found @',states.length-idx-1)
        // console.log('match info:',{state:state.current,condition,prevState})
        time = state.time;
        timeIdx = states.length - idx;
        found = true;
        savedMatches = currentMatches;
        return false;
      } else {
        // console.log('some other condition',{currentMatch, currentMatches,pastMatch,found})
      }
    });
  return [time, found, timeIdx];
};

const insertEventStepAtTime = (time, idx, states, eventStep, condition) => {
  // Find state at time
  let currentState = compileStates(states, time);
  // Merge with new eventStep's effects
  currentState = lodash.merge(currentState, eventStep.effect);
  states.splice(idx, 0, {
    time: time + eventStep.delay,
    type: eventStep.stepType,
    effect: eventStep.effect,
    condition: condition,
    data: eventStep.data,
    source: eventStep.source,
    current: eventStep.effect,
  });
  return withRefreshedStates(states);
};

export const eventsToStates = (events) => {
  let tempStates = [];
  let pendingSet = [];
  events.forEach((event, i) => {
    const allEffects = compileEffects(event.onTrigger);
    const [currentTime, found, timeIdx] = searchForTime(
      tempStates,
      event.condition,
      allEffects
    );
    if (found || tempStates.length === 0) {
      // If matching and found, add to the stack at that time.
      event.onTrigger.forEach((eventStep, i) => {
        tempStates = insertEventStepAtTime(
          currentTime,
          timeIdx + i,
          tempStates,
          eventStep,
          event.condition
        );
      });
      // Check pending for any events that can be added
      pendingSet.forEach((pendingEvent) => {
        const allPendingEffects = compileEffects(pendingEvent.onTrigger);
        const [currentPendingTime, pendingFound, pendingTimeIdx] =
          searchForTime(tempStates, pendingEvent.condition, allPendingEffects);
        if (pendingFound) {
          // console.log('adding event from pending',event)
          pendingEvent.onTrigger.forEach((pendingEventStep, j) => {
            tempStates = insertEventStepAtTime(
              currentPendingTime,
              pendingTimeIdx + j,
              tempStates,
              pendingEventStep,
              pendingEvent.condition
            );
            // tempStates.sort((a,b)=>a.time - b.time);
          });
        }
      });
    } else {
      // Event couldn't be resolved, so add to the pending
      pendingSet.push(event);
    }
  });
  return tempStates;
};

export const statesToSteps = (states) =>
  states.map((state) => ({
    ...lodash.omit(state, "delay", "effect", "current", "condition"),
  }));
