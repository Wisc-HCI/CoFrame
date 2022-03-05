import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { STATUS, STEP_CALCULATOR } from "../Constants";
import { ContainerIconStyled } from "./icons";
//import {Tasks} from 'grommet-icons';

export const inputOutputType = {
  name: 'Input / Output',
  type: TYPES.OBJECT,
  referenceBlock: {
    onCanvas: false,
    color: '#0072b2',
    icon: ContainerIconStyled
  },
  properties: {
    description: {
      name: 'Description',
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: "",
      isList: false,
      fullWidth: true
    },
    relativeObject: {
      name: 'Relative to',
      accepts: ["thingType", "machineType"],
      default: null,
      isList: false
    },   
    position: {
      name: 'Position',
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: { x: 0, y: 0, z: 0 },
      isList: false,
      fullWidth: true
    },
    rotation: {
      name: 'Rotation',
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: { x: 0, y: 0, z: 0, w: 0 },
      isList: false,
      fullWidth: true
    },
    thing: {
      name: "Thing",
      accepts: ["thingType"],
      default: null,
      isList: false,
      nullValid: true
    },
    status: {
      name: 'Status',
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: STATUS.PENDING
    },
    computeSteps: {
      name: 'Compute Steps',
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: STEP_CALCULATOR.NULL
    },
    steps: {
      name: 'Steps',
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: {}
    }
  }
}