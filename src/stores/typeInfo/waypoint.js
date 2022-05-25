import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { WaypointIconStyled } from "./icons";
import { FiMoreHorizontal, FiAlertTriangle, FiAlertOctagon, FiRefreshCw, FiThumbsUp } from "react-icons/fi";
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";
import './rotate.css'

export const waypointType = {
    name: 'Waypoint',
    type: TYPES.OBJECT,
    instanceBlock: null,
    referenceBlock: {
      onCanvas: false,
      color: "#AD1FDE",
      icon: WaypointIconStyled,
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
      description: {
        name: 'Description',
        type: SIMPLE_PROPERTY_TYPES.IGNORED, 
        default: "",
        isList: false,
        fullWidth: true
      },
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
      status: {
        name: 'Status',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: STATUS.PENDING
      },
      compileFn: {
        name: 'Compile Function',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: COMPILE_FUNCTIONS.POSE
      },
      compiled: {
        name: 'Compiled',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: {}
      },
      updateFields: {
        name: 'Update Fields',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: ['position','rotation']
      },
      singleton: {
        name: 'singleton',
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: true
      }
    }
  }