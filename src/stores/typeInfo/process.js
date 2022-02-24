import { TYPES, SIMPLE_PROPERTY_TYPES,EXTRA_TYPES } from "simple-vp";

export const processType = {
  name: 'Process',
  type: TYPES.OBJECT,
  instanceBlock: null,
  referenceBlock: {
    onCanvas: false,
    color: '#888888',
    // icon: ,
    extras: [
      EXTRA_TYPES.LOCKED_INDICATOR,
      EXTRA_TYPES.DELETE_BUTTON
    ]
  },
  properties: {
    description: {
      name: 'Description',
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: "",
      isList: false,
      fullWidth: true,
      nullValid: true
    },
    machine: {
      name: 'Machine',
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: "",
      isList: false,
      fullWidth: true
    },
    processTime: { 
      name: 'Process Time',
      name: 'Duration',
      type: SIMPLE_PROPERTY_TYPES.NUMBER,
      default: 0,
      min: 0,
      max: null
    },
    inputs: {
      name: 'Inputs',
      default: [],
      accepts: ["inputOutputType"],
      isList: true,
      fullWidth: true
    },
    outputs: {
      name: 'Outputs',
      default: [],
      accepts: ["inputOutputType"],
      isList: true,
      fullWidth: true
    },
  }
}