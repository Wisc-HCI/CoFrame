
import { SIMPLE_PROPERTY_TYPES } from "open-vp";
import { STATUS, ERROR } from "../Constants";
import "./rotate.css";

export const baseTypeData = {
    properties: {
      description: {
        name: "Description",
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        isList: false,
        fullWidth: true,
      },
      status: {
        name: "Status",
        type: SIMPLE_PROPERTY_TYPES.METADATA,
        default: STATUS.PENDING,
      },
      pendingChanges: {
        name: "Pending Changes",
        type: SIMPLE_PROPERTY_TYPES.METADATA,
        default: 0,
      },
      errorCode: {
        name: "Error Code",
        type: SIMPLE_PROPERTY_TYPES.METADATA,
        nullValid: true,
        default: null
      },
      compileFn: {
        name: "Compile Function",
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
      },
      updateFields: {
        name: "Update Fields",
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
      },
      singleton: {
        name: "singleton",
        type: SIMPLE_PROPERTY_TYPES.IGNORED,
        default: false,
      },
    },
  };

  export const baseIndicatorLabelFn = (data)=>{
    const status = data.properties?.status ? data.properties.status : data.refData?.properties?.status;
    const errorCode = data.properties?.errorCode ? data.properties.errorCode : data.refData?.properties?.errorCode;
    try {
      switch (status) {
        case STATUS.PENDING:
          return "Updating";
        case STATUS.VALID:
          return "Valid";
        default:
          switch (errorCode) {
            case ERROR.MISSING_PARAMETER: 
              return "Missing required parameter";
            case ERROR.INVALID_PARAMETER:
              return "Invalid parameter";
            case ERROR.MISMATCHED_GIZMO: 
              return 'Process requires a different gizmo';
            case ERROR.UNREACHABLE_POSE: 
              return 'Pose is unreachable';
            case ERROR.TRAJECTORY_PROGRESS: 
              return 'Trajectory computation could not proceed';
            case ERROR.TIMEOUT: 
              return 'Compilation timed out';
            case ERROR.CHILD_FAILED: 
              return 'Failed due to failed inner content';
            case ERROR.DOES_NOTHING: 
              return 'This action does nothing';
          };
      }
    } catch {
      console.warn(`Error getting status with block of type ${data.type}`)
      return "Status"
    }
    
  }