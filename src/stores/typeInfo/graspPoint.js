import { TYPES, SIMPLE_PROPERTY_TYPES, EXTRA_TYPES } from "open-vp";
import { baseTypeData } from "./baseType";
import { COMPILE_FUNCTIONS } from "../Constants";
import { RiSpace } from "react-icons/ri";
import { merge } from "lodash";

const graspDoc = "An offset from a [Thing](thingType) or [Tool](toolType) that a [Gripper](gripperAgentType) can grasp"

const graspPoint = {
  name: "Grasp Point",
  type: TYPES.OBJECT,
  description: graspDoc,
  instanceBlock: null,
  referenceBlock: {
    onCanvas: false,
    color: "#6663f0",
    icon: RiSpace,
    extras: [EXTRA_TYPES.SELECTION_TOGGLE,EXTRA_TYPES.DOC_TOGGLE],
  },
  properties: {
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
      default: { x: 0, y: 0, z: 0 },
      isList: false,
      fullWidth: true,
    },
    gripDistance: {
      name: "Grip Distance",
      type: SIMPLE_PROPERTY_TYPES.NUMBER,
      default: 50,
      isList: false
    },
    updateFields: {
      default: ["gripDistance", "rotation", "position"]
    },
    singleton: {
      default: true,
    },
    compileFn: {
      default: COMPILE_FUNCTIONS.PROPERTY
    },
  }
};

export const graspPointType = merge(graspPoint, baseTypeData);
