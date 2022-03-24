import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { ContainerIconStyled } from "./icons";
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";

export const hierarchicalType = {
    name: "Hierarchical",
    type: TYPES.OBJECT,
    instanceBlock: {
      hideNewPrefix: true,
      onCanvas: false,
      color: '#7f7f7f',
      icon: ContainerIconStyled,
      extras: [
        EXTRA_TYPES.COLLAPSE_TOGGLE,
        { 
          type: EXTRA_TYPES.INDICATOR,
          accessor: (data)=>data.properties.children.length,
          label: 'Size'
        },
        EXTRA_TYPES.LOCKED_INDICATOR
      ]
    },
    referenceBlock: null,
    properties: {
      description: {
        name: 'Description',
        type: SIMPLE_PROPERTY_TYPES.IGNORED, 
        default: "",
        isList: false,
        fullWidth: true
      },
      children: {
        name: 'Children',
        accepts: ['hierarchicalType', 'skillType', 'delayType', 'breakpointType', 'gripperType', 'machineInitType', 'processStartType', 'processStopType', 'processWaitType', 'moveTrajectoryType', 'moveUnplannedType','robotInitType'],
        default: [],
        isList: true,
        fullWidth: true
      },
      status: {
        name: 'Status',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: STATUS.PENDING
      },
      compileFn: {
        name: 'Compile Function',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: COMPILE_FUNCTIONS.SIMPLE
      },
      compiled: {
        name: 'Compiled',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: {}
      }
    }
  }