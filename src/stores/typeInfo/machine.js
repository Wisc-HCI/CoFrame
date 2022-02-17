import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { MachineIconStyled } from "./icons";
import { FiMoreHorizontal } from "react-icons/fi";

export const machineType = {
    name: 'Machine',
    type: TYPES.OBJECT,
    instanceBlock: null,
    referenceBlock: {
      onCanvas: false,
      color: "#B3A533",
      icon: MachineIconStyled,
      extras: [
        EXTRA_TYPES.LOCKED_INDICATOR,
        {
          icon: FiMoreHorizontal,
          type: EXTRA_TYPES.DROPDOWN,
          contents: [
            EXTRA_TYPES.DELETE_BUTTON,
            EXTRA_TYPES.DEBUG_TOGGLE
          ]
        }
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
      tf: {
        name: 'Transform Frame',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: "",
        isList: false,
        fullWidth: true
      },
      mesh: {
        name: 'Mesh',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: "",
        isList: false,
        fullWidth: true
      },
      collisionMesh: {
        name: 'Collision Mesh',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: "",
        isList: false,
        fullWidth: true
      }
    }
  }