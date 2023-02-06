
import { SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { STATUS } from "../Constants";
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