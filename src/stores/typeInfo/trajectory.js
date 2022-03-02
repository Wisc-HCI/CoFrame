import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { ContainerIconStyled } from "./icons";
import { STATUS, STEP_CALCULATOR } from "../Constants";

export const trajectoryType = {
    name: 'Trajectory',
    type: TYPES.OBJECT,
    instanceBlock: {
      hideNewPrefix: true,
      onCanvas: false,
      color: '#c5329a',
      icon: ContainerIconStyled,
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
        fullWidth: true
      },
      startLocation: {
        name: "Start Location",
        accepts: ["locationType"],
        default: null,
        isList: false
      },
      waypoints: {
        name: "Waypoints",
        accepts: ["waypointType"],
        default: [],
        isList: true
      },
      endLocation: {
        name: "End Location",
        accepts: ["locationType"],
        default: null,
        isList: false
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