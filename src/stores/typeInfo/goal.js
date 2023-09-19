import { TYPES, SIMPLE_PROPERTY_TYPES, EXTRA_TYPES } from "open-vp";
import { baseTypeData } from "./baseType";
import { COMPILE_FUNCTIONS } from "../Constants";
import { RiSpace } from "react-icons/ri";
import { merge } from "lodash";

const goalDoc = 'A special structure block that is used to define possible implementations for tasks. It accepts the same types of blocks as the [Program](programType), as well as some additional information about its completion conditions and status';

const goal = {
  name: "Program Goal",
  type: TYPES.OBJECT,
  description: goalDoc,
  instanceBlock: null,
  referenceBlock: {
    onCanvas: false,
    color: "#6663f0",
    icon: RiSpace,
    extras: [EXTRA_TYPES.SELECTION_TOGGLE,EXTRA_TYPES.DOC_TOGGLE],
  },
  properties: {
    header: {
      name: "Header Text",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: ""
    },
    textfield: {
      name: "Display Text",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: "",
    },
    condition: {
      name: "Accepting Condition",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: {},
    },
    example: {
      name: "Example",
      accepts: [
        "hierarchicalType",
        "skillType",
        "delayType",
        "breakpointType",
        "moveGripperType",
        "machineInitType",
        "processStartType",
        "processStopType",
        "processWaitType",
        "moveTrajectoryType",
        "moveUnplannedType",
        "robotInitType",
      ],
      default: [],
      isList: true,
    },
    isComplete: {
      name: "Condition has passed",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: false,
    },
    tab: {
      name: "Program tab to focus",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: "default",
    },
    updateFields: {
      default: ["header", "textfield", "condition", "example"]
    },
    singleton: {
      default: true,
    },
    compileFn: {
      default: COMPILE_FUNCTIONS.NULL
    },
  }
};

export const goalType = merge(goal, baseTypeData);
