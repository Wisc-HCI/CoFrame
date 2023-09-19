import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "open-vp";
import { ContainerIconStyled } from "./icons";
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";
import { FiMoreHorizontal } from "react-icons/fi";
import { baseTypeData } from "./baseType";
import { merge } from "lodash";

const hierarchicalDoc = "A hierarchical is a special type of action that acts as a grouping or organization structure within enclosing actions. While they don't have any utility in terms of changing the flow of a [Program](programType), they can be named and collapsed for easier viewing and editing.";

const hierarchicalFeatures = {
    name: "Hierarchical",
    description: hierarchicalDoc,
    type: TYPES.OBJECT,
    instanceBlock: {
      hideNewPrefix: true,
      onCanvas: false,
      color: '#7f7f7f',
      icon: ContainerIconStyled,
      extras: [
        EXTRA_TYPES.COLLAPSE_TOGGLE,
        {
          icon: FiMoreHorizontal,
          type: EXTRA_TYPES.DROPDOWN,
          contents: [
            EXTRA_TYPES.DELETE_BUTTON,
            EXTRA_TYPES.DOC_TOGGLE,
            EXTRA_TYPES.SELECTION_TOGGLE
          ],
        },
        { 
          type: EXTRA_TYPES.INDICATOR_TEXT,
          accessor: (data)=>data.properties.children.length,
          label: 'Size'
        },
        EXTRA_TYPES.LOCKED_INDICATOR
      ]
    },
    referenceBlock: null,
    properties: {
      children: {
        name: 'Children',
        accepts: ['hierarchicalType', 'skillType', 'delayType', 'breakpointType', 'moveGripperType', 'machineInitType', 'processStartType', 'processStopType', 'processWaitType', 'moveTrajectoryType', 'moveUnplannedType','robotInitType'],
        default: [],
        isList: true,
        fullWidth: true
      },
      compileFn: {
        default: COMPILE_FUNCTIONS.SIMPLE
      },
      updateFields: {
        default: ['children']
      }
    }
  }

  export const hierarchicalType = merge(hierarchicalFeatures, baseTypeData);