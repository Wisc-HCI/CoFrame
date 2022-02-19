import { TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";

export const processType = {
    name: 'process',
    type: TYPES.OBJECT,
    instanceBlock: null,
    referenceBlock: null,
    properties: {
      description: {
        name: 'Description',
        type: SIMPLE_PROPERTY_TYPES.IGNORED, 
        default: "",
        isList: false,
        fullWidth: true
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
        type: SIMPLE_PROPERTY_TYPES.IGNORED, 
        default: 0,
        isList: false,
        fullWidth: true
      },
      outputs: {
        name: 'Outputs',
        type: SIMPLE_PROPERTY_TYPES.IGNORED, 
        default: [],
        isList: true,
        fullWidth: true
      },
      inputs: {
        name: 'Inputs',
        type: SIMPLE_PROPERTY_TYPES.IGNORED, 
        default: [],
        isList: true,
        fullWidth: true
      }
    }
  }