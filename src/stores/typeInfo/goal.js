import { TYPES, SIMPLE_PROPERTY_TYPES, EXTRA_TYPES } from "simple-vp";
import { baseTypeData } from "./baseType";
import { COMPILE_FUNCTIONS } from "../Constants";
import { RiSpace } from "react-icons/ri";
import { merge } from "lodash";

const goal = {
  name: "Program Goal",
  type: TYPES.OBJECT,
  instanceBlock: null,
  referenceBlock: {
    onCanvas: false,
    color: "#6663f0",
    icon: RiSpace,
    extras: [EXTRA_TYPES.SELECTION_TOGGLE],
  },
  properties: {
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
    isComplete: {
        name: "Condition has passed",
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: false,
    },
    updateFields: {
      default: ["textfield", "condition"]
    },
    singleton: {
      default: true,
    },
    compileFn: {
      default: COMPILE_FUNCTIONS.PROPERTY
    },
  }
};

export const goalType = merge(goal, baseTypeData);
