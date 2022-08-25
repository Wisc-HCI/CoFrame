import { STEP_TYPE } from "../stores/Constants";

const EVENT_TYPES = [STEP_TYPE.LANDMARK];
const START_TYPES = [STEP_TYPE.ACTION_START, STEP_TYPE.PROCESS_START];
const END_TYPES = [STEP_TYPE.ACTION_END, STEP_TYPE.PROCESS_END];
const ITEM_TYPES = [STEP_TYPE.SPAWN_ITEM, STEP_TYPE.DESTROY_ITEM];

const capitalize = (row) => row.charAt(0).toUpperCase() + row.slice(1);

export function smoothInterpolateScalar(x, y) {
  //const defaultFn = (v) => 0;
  if (x.length <= 0) {
    return null;
  }
  const interp = (v) => {
    const xi = v > x[x.length - 1] ? v % x[x.length - 1] : v;
    let lastY = 0;
    let lastX = 0;
    for (let i = 0; i < x.length; i++) {
      if (x[i] <= xi) {
        lastX = x[i];
        lastY = y[i];
      } else {
        lastY = ((y[i] - lastY) * (xi - lastX)) / (x[i] - lastX) + lastY;
        break;
      }
    }
    return lastY;
  };
  return interp;
}

export const stepDataToBlocksAndTracks = (
  programData,
  programSpec,
  focusSteps,
  trackTypes,
  robot
) => {
  let blockData = [];
  let eventData = [];
  let openSteps = {};
  [...focusSteps, { type: null, time: Number.POSITIVE_INFINITY }].forEach(
    (step, i) => {
      let track = robot.id;
      let label = step?.data?.label ? step.data.label : "";
      let event = "action";
      if (step?.data?.process) {
        track = step.data.process;
        event = "process";
      }
      if (step?.data?.machine) {
        track = step.data.machine;
        event = "machines";
      }
      if (step?.data?.tool) {
        track = step.data.tool;
        event = "tools";
      }
      if (!trackTypes[track]) {
        const objType = programData[track].type;
        // const objName = programData[track].name;
        trackTypes[track] = {
          label: programData[track].name,
          color: programSpec.objectTypes[objType].referenceBlock.color
        };
      }
      // console.log(step)
      if (START_TYPES.includes(step.type)) {
        const label =
          step.type === STEP_TYPE.ACTION_START
            ? programData[step.source].name
            : programData[step.data.process].name;
        openSteps[step.source] = {
          track: track,
          event,
          label,
          start: step.time,
          end: step.time + 1
        };
      } else if (END_TYPES.includes(step.type)) {
        openSteps[step.source].end = step.time;
        blockData.push(openSteps[step.source]);
        delete openSteps[step.source];
      } else if (EVENT_TYPES.includes(step.type)) {
        if (step.data.process) {
          label = step.data.label.replace("Process", programData[track].name);
        }
        if (step.data.machine) {
          label = step.data.label.replace("Machine", programData[track].name);
        }
        if (step.data.tool) {
          label = step.data.label.replace("Tool", programData[track].name);
        }
        eventData.push({
          time: step.time,
          track,
          event,
          label
        });
      } else if (ITEM_TYPES.includes(step.type)) {
        eventData.push({
          time: step.time,
          track,
          event: "things",
          label: `${
            programData[step.data.thing]
              ? programData[step.data.thing].name
              : "Thing"
          } ${step.type === STEP_TYPE.SPAWN_ITEM ? "Spawned" : "Consumed"}`
        });
      }
    }
  );
  return [blockData, eventData, trackTypes];
};

export const interpolateScalarSmooth = (x, y) => {
  //const defaultFn = (v) => 0;
  if (x.length <= 0) {
    return null;
  }
  const interp = (v) => {
    const xi = v > x[x.length - 1] ? v % x[x.length - 1] : v;
    let lastY = 0;
    let lastX = 0;
    for (let i = 0; i < x.length; i++) {
      if (x[i] <= xi) {
        lastX = x[i];
        lastY = y[i];
      } else {
        lastY = ((y[i] - lastY) * (xi - lastX)) / (x[i] - lastX) + lastY;
        break;
      }
    }
    return lastY;
  };
  return interp;
}

export const collapseSeries = (series, keys) =>
  series.map((s) => ({ x: s.x, y: Math.max(...keys.map((k) => s[k])) }));

export const getIssueBlocksForThreshold = (series, threshold) => {
  let blocks = [];
  let open = false;
  let lastBlock = null;
  let lastData = null;
  series.forEach((s) => {
    if (!lastData && s.y >= threshold.range[0] && s.y <= threshold.range[1]) {
      // Series data is within range, so start a new block series
      // console.log("series in range");
      lastBlock = { x0: 0, x1: 0 };
      open = true;
    } else if (
      s.y >= threshold.range[0] &&
      s.y <= threshold.range[1] &&
      !open
    ) {
      // The series has started to become in range.
      // console.log("series in range");

      // const changeX = findChange(lastData, s, threshold, descriptor);
      // console.log("open", changeX);
      lastBlock = { x0: s.x, x1: s.x };
      open = true;
    } else if ((s.y < threshold.range[0] || s.y > threshold.range[1]) && open) {
      // The series has gone out of range
      // console.log("series out of range");
      // const changeX = findChange(lastData, s, threshold, descriptor);
      blocks.push({ ...lastBlock, x1: s.x });
      lastBlock = null;
      open = false;
      // descriptor = s.y < threshold.range[0] ? "below" : "above";
      // console.log(`series out of range`);
    }
    lastData = s;
  });
  if (open) {
    // console.log('hanging')
    blocks.push({ ...lastBlock, x1: lastData.x });
  }
  return blocks;
};

export const collapseBands = (blockData, eventData, trackTypes, expanded) => {
  let filteredBlockData = [];
  let filteredEventData = [];
  let filteredTrackTypes = {};
  let inProgress = {};
  blockData.sort((a, b) => a.start - b.start);
  blockData.forEach((block) => {
    if (expanded.includes(block.event)) {
      // console.log("in-expanded", block);
      filteredBlockData.push(block);
      if (!filteredTrackTypes[block.track]) {
        filteredTrackTypes[block.track] = trackTypes[block.track];
      }
    } else {
      // console.log("in-collapsed", block);
      let existingBlock = inProgress[block.event];
      if (existingBlock) {
        if (existingBlock.end < block.start) {
          filteredBlockData.push(existingBlock);
          if (!filteredTrackTypes[block.track]) {
            filteredTrackTypes[block.track] = {
              ...trackTypes[block.event],
              label: capitalize(block.event)
            };
          }
          delete inProgress[block.event];
          inProgress[block.event] = { ...block, track: block.event, label: "" };
        } else {
          inProgress[block.event].end = block.end;
        }
      } else {
        inProgress[block.event] = { ...block, track: block.event, label: "" };
      }
    }
  });
  Object.values(inProgress).forEach((ip) => {
    filteredBlockData.push(ip);
    if (!filteredTrackTypes[ip.track]) {
      filteredTrackTypes[ip.event] = {
        ...trackTypes[ip.event],
        label: capitalize(ip.event)
      };
    }
  });
  eventData.forEach((event) => {
    if (expanded.includes(event.event)) {
      filteredEventData.push(event);
      if (!filteredTrackTypes[event.track]) {
        filteredTrackTypes[event.track] = trackTypes[event.track];
      }
    } else {
      filteredEventData.push({ ...event, track: event.event });
      if (!filteredTrackTypes[event.event]) {
        filteredTrackTypes[event.event] = {
          ...trackTypes[event.track],
          label: capitalize(event.event)
        };
      }
    }
  });
  // console.log({ filteredBlockData, filteredEventData, filteredTrackTypes });
  return [filteredBlockData, filteredEventData, filteredTrackTypes];
};

export const getMaxForAllSeries = (series, keys) =>
  Math.max(...series.map((s) => Math.max(...keys.map((k) => s[k]))));
export const getMinForAllSeries = (series, keys) =>
  Math.min(...series.map((s) => Math.min(...keys.map((k) => s[k]))));

export const getBlocks = (series, threshold) => {
  let blocks = [];
  let open = false;
  let lastBlock = null;
  let lastData = null;
  series.forEach((s) => {
    if (!lastData && s.y >= threshold.range[0] && s.y <= threshold.range[1]) {
      // Series data is within range, so start a new block series
      // console.log("series in range");
      lastBlock = { x0: 0, x1: 0 };
      open = true;
    } else if (
      s.y >= threshold.range[0] &&
      s.y <= threshold.range[1] &&
      !open
    ) {
      // The series has started to become in range.
      // console.log("series in range");

      // const changeX = findChange(lastData, s, threshold, descriptor);
      // console.log("open", changeX);
      lastBlock = { x0: s.x, x1: s.x };
      open = true;
    } else if ((s.y < threshold.range[0] || s.y > threshold.range[1]) && open) {
      // The series has gone out of range
      // console.log("series out of range");
      // const changeX = findChange(lastData, s, threshold, descriptor);
      blocks.push({ ...lastBlock, x1: s.x });
      lastBlock = null;
      open = false;
      // descriptor = s.y < threshold.range[0] ? "below" : "above";
      // console.log(`series out of range`);
    }
    lastData = s;
  });
  if (open) {
    // console.log('hanging')
    blocks.push({ ...lastBlock, x1: lastData.x });
  }
  return blocks;
};