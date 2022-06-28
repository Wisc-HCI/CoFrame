import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { LocationIconStyled } from "./icons";
import { FiMoreHorizontal, FiAlertTriangle, FiAlertOctagon, FiRefreshCw, FiThumbsUp } from "react-icons/fi";
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";
import './rotate.css'
import { baseTypeData } from "./baseType";
import { merge } from "lodash";

const locationFeatures = {
  name: 'Location',
  type: TYPES.OBJECT,
  instanceBlock: null,
  referenceBlock: {
    onCanvas: false,
    color: "#8624E0",
    icon: LocationIconStyled,
    extras: [
      EXTRA_TYPES.LOCKED_INDICATOR,
      { 
        type: EXTRA_TYPES.INDICATOR_ICON,
        accessor: (data)=>{
          if (data.refData.properties.status===STATUS.FAILED) {
            return <FiAlertOctagon color='white' fill='red'/>
          } else if (data.refData.properties.status===STATUS.VALID) {
            return <FiThumbsUp color='white'/>
          } else if (data.refData.properties.status===STATUS.WARN) {
            return <FiAlertTriangle color='white' fill='#ff7300'/>
          }else if (data.refData.properties.status===STATUS.PENDING) {
            return <FiRefreshCw className='rotate'/>
          }
        },
        label: 'Status'
      },
      {
        icon: FiMoreHorizontal,
        type: EXTRA_TYPES.DROPDOWN,
        contents: [
          EXTRA_TYPES.NAME_EDIT_TOGGLE,
          EXTRA_TYPES.DELETE_BUTTON,
          EXTRA_TYPES.DEBUG_TOGGLE,
          EXTRA_TYPES.SELECTION_TOGGLE
        ]
      }
    ]
  },
  properties: {
    position: {
      name: 'Position',
      type: SIMPLE_PROPERTY_TYPES.IGNORED, 
      default: {x: null, y: null, z: null},
      isList: false,
      fullWidth: true
    },
    rotation: {
      name: 'Rotation',
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: {x: null, y: null, z: null, w: null},
      isList: false,
      fullWidth: true
    },
    states: {
      name: 'States',
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: {}
    },
    reachability: {
      name: 'Reachability',
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: {}
    },
    compileFn: {
      default: COMPILE_FUNCTIONS.POSE
    },
    updateFields: {
      default: ['position','rotation']
    },
    singleton: {
      default: true
    }
  }
}

export const locationType = merge(locationFeatures, baseTypeData);