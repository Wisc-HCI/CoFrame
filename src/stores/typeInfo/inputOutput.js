import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";
import { InputOutputIconStyled } from "./icons";
import { FiMoreHorizontal } from "react-icons/fi";

export const inputOutputType = {
  name: "Input / Output",
  type: TYPES.OBJECT,
  referenceBlock: {
    onCanvas: false,
    color: "#0072b2",
    icon: InputOutputIconStyled,
    extras: [
      EXTRA_TYPES.LOCKED_INDICATOR,
      {
        icon: FiMoreHorizontal,
        type: EXTRA_TYPES.DROPDOWN,
        contents: [
          EXTRA_TYPES.DELETE_BUTTON,
          EXTRA_TYPES.DEBUG_TOGGLE,
          EXTRA_TYPES.SELECTION_TOGGLE,
        ],
      },
    ],
  },
  properties: {
    description: {
      name: "Description",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: "",
      isList: false,
      fullWidth: true,
    },
    relativeTo: {
      name: "Relative to",
      accepts: ["thingType", "machineType", "toolType"],
      default: null,
      isList: false,
    },
    position: {
      name: "Position",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: { x: 0, y: 0, z: 0 },
      isList: false,
      fullWidth: true,
    },
    rotation: {
      name: "Rotation",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: { x: 0, y: 0, z: 0, w: 0 },
      isList: false,
      fullWidth: true,
    },
    thing: {
      name: "Thing",
      accepts: ["thingType", "toolType"],
      default: null,
      isList: false,
      nullValid: true,
    },
    status: {
      name: "Status",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: STATUS.PENDING,
    },
    compileFn: {
      name: "Compile Function",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: COMPILE_FUNCTIONS.PROPERTY,
    },
    compiled: {
      name: "Compiled",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: {},
    },
    updateFields: {
      name: "Update Fields",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: ["relativeTo", "position", "rotation", "thing"],
    },
    singleton: {
      name: "singleton",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: true,
    },
  },
};
