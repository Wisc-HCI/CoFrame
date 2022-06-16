import { STEP_TYPE } from "../stores/Constants";

const EVENT_TYPES = [STEP_TYPE.LANDMARK];
const START_TYPES = [STEP_TYPE.ACTION_START, STEP_TYPE.PROCESS_START];
const END_TYPES = [STEP_TYPE.ACTION_END, STEP_TYPE.PROCESS_END];
const ITEM_TYPES = [STEP_TYPE.SPAWN_ITEM, STEP_TYPE.DESTROY_ITEM];

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
