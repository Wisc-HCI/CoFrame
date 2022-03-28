import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { ContainerIconStyled } from "./icons";
import { FiMoreHorizontal } from "react-icons/fi";
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";

export const programType = {
    name: 'Program',
    type: TYPES.OBJECT,
    instanceBlock: {
      hideNewPrefix: true,
      onCanvas: true,
      color: "#3f3f3f",
      icon: ContainerIconStyled,
      extras: [
        {
          icon: FiMoreHorizontal,
          type: EXTRA_TYPES.DROPDOWN,
          contents: [
            EXTRA_TYPES.NAME_EDIT_TOGGLE,
            EXTRA_TYPES.LOCKED_INDICATOR,
            EXTRA_TYPES.SELECTION_TOGGLE,
            EXTRA_TYPES.DEBUG_TOGGLE,
            { 
              type: EXTRA_TYPES.INDICATOR,
              accessor: (data)=>{
                if (data.properties.status===STATUS.FAILED) {
                  return 'Failure'
                } else if (data.properties.status===STATUS.VALID) {
                  return 'Valid'
                } else if (data.properties.status===STATUS.PENDING) {
                  return 'Pending'
                }
              },
              label: 'Status'
            },
            { 
              type: EXTRA_TYPES.INDICATOR,
              accessor: (data)=>data.properties.children.length,
              label: 'Size'
            }
          ]
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
      },
      updateFields: {
        name: 'Update Fields',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: ['children']
      }
    }
  }