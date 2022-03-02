import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { SkillIconStyled } from "./icons";
import { FiMoreHorizontal } from "react-icons/fi";
import { STATUS, STEP_CALCULATOR } from "../Constants";

export const skillType = {
    name: 'Skill',
    type: TYPES.FUNCTION,
    instanceBlock: {
      hideNewPrefix: false,
      onCanvas: true,
      color: "#62869e",
      icon: SkillIconStyled,
      extras: [
        EXTRA_TYPES.LOCKED_INDICATOR,
        {
          icon: FiMoreHorizontal,
          type: EXTRA_TYPES.DROPDOWN,
          contents: [
            EXTRA_TYPES.SELECTION_TOGGLE,
            EXTRA_TYPES.DELETE_BUTTON,
            EXTRA_TYPES.LOCKED_INDICATOR,
            EXTRA_TYPES.DEBUG_TOGGLE,
            {
              type: EXTRA_TYPES.ADD_ARGUMENT_GROUP,
              allowed: ['machineType', 'locationType', 'thingType', 'trajectoryType']
            }
          ]
        }
      ]
    },
    callBlock: {
      onCanvas: false,
      color: "#62869e",
      icon: SkillIconStyled
    },
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
        accepts: ['hierarchicalType', 'skillCallType', 'delayType', 'breakpointType', 'gripperType', 'machineInitType', 'processStartType', 'processStopType', 'processWaitType', 'moveTrajectoryType', 'moveUnplannedType'],
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
        default: STEP_CALCULATOR.SKILL
      },
      steps: {
        name: 'Steps',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: {}
      }
    }
  }