import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { ContainerIconStyled } from "./icons";
import { FiMoreHorizontal, FiAlertTriangle, FiAlertOctagon, FiRefreshCw, FiThumbsUp } from "react-icons/fi";
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";
import './rotate.css'

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
          type: EXTRA_TYPES.INDICATOR_ICON,
          accessor: (data)=>{
            if (data.properties.status===STATUS.FAILED) {
              return <FiAlertOctagon color='white' fill='red'/>
            } else if (data.properties.status===STATUS.VALID) {
              return <FiThumbsUp color='white'/>
            } else if (data.properties.status===STATUS.WARN) {
              return <FiAlertTriangle color='white' fill='#ff7300'/>
            }else if (data.properties.status===STATUS.PENDING) {
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
            EXTRA_TYPES.LOCKED_INDICATOR,
            EXTRA_TYPES.SELECTION_TOGGLE,
            EXTRA_TYPES.DEBUG_TOGGLE,
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
        accepts: ['hierarchicalType', 'skillType', 'delayType', 'breakpointType', 'moveGripperType', 'machineInitType', 'processStartType', 'processStopType', 'processWaitType', 'moveTrajectoryType', 'moveUnplannedType','robotInitType'],
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