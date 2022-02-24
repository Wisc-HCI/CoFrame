import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { ContainerIconStyled } from "./icons";
import { FiMoreHorizontal } from "react-icons/fi";
import { STATUS, STEP_CALCULATOR } from "../Constants";

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
          type: EXTRA_TYPES.INDICATOR,
          accessor: (data)=>data.properties.children.length,
          label: 'Size'
        },
        {
          icon: FiMoreHorizontal,
          type: EXTRA_TYPES.DROPDOWN,
          contents: [
            EXTRA_TYPES.NAME_EDIT_TOGGLE,
            EXTRA_TYPES.LOCKED_INDICATOR,
            EXTRA_TYPES.SELECTION_TOGGLE,
            {
              icon: FiMoreHorizontal,
              label: 'More Options',
              type: EXTRA_TYPES.DROPDOWN,
              contents: [
                EXTRA_TYPES.NAME_EDIT_TOGGLE,
                EXTRA_TYPES.COLLAPSE_TOGGLE,
                EXTRA_TYPES.LOCKED_INDICATOR,
                { 
                  type: EXTRA_TYPES.INDICATOR,
                  accessor: (data)=>data.properties.children.length,
                  label: 'Size'
                }
              ]
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
        accepts: ['hierarchicalType', 'skillType', 'delayType', 'breakpointType', 'gripperType', 'machineInitType', 'processStartType', 'processStopType', 'processWaitType', 'moveTrajectoryType', 'moveUnplannedType'],
        default: [],
        isList: true,
        fullWidth: true
      },
      status: {
        name: 'Status',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: STATUS.PENDING
      },
      computeSteps: {
        name: 'Compute Steps',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: STEP_CALCULATOR.SIMPLE
      },
      steps: {
        name: 'Steps',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: []
      }
    }
  }