import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "open-vp";
import { ContainerIconStyled, statusIcon } from "./icons";
import {
  FiMoreHorizontal
} from "react-icons/fi";
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";
import "./rotate.css";
import { baseTypeData } from "./baseType";
import { merge } from "lodash";

const programDoc = `The Program is the main sequence of actions, such as [Hierarchicals](hierarchicalType), [Move Trajectory](moveTrajectoryType), or [Process Start](processStartType), and [Skills](skillType) that are executed by the [Robot](robotAgentType).`;

const programFeatures = {
  name: "Program",
  type: TYPES.OBJECT,
  description: programDoc,
  instanceBlock: {
    hideNewPrefix: true,
    onCanvas: true,
    color: "#3f3f3f",
    icon: ContainerIconStyled,
    extras: [
      {
        type: EXTRA_TYPES.INDICATOR_ICON,
        accessor: statusIcon,
        label: "Status",
      },
      EXTRA_TYPES.DOC_TOGGLE,
      {
        icon: FiMoreHorizontal,
        type: EXTRA_TYPES.DROPDOWN,
        contents: [
          EXTRA_TYPES.NAME_EDIT_TOGGLE,
          EXTRA_TYPES.LOCKED_INDICATOR,
          EXTRA_TYPES.SELECTION_TOGGLE,
          {
            type: EXTRA_TYPES.INDICATOR_TEXT,
            accessor: (data) => data.properties.children.length,
            label: "Size",
          },
        ],
      }
    ],
  },
  referenceBlock: null,
  properties: {
    description: {
      name: "Description",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: "",
      isList: false,
      fullWidth: true,
    },
    children: {
      name: "Children",
      accepts: [
        "hierarchicalType",
        "skillType",
        "delayType",
        "breakpointType",
        "moveGripperType",
        "machineInitType",
        "processStartType",
        "processStopType",
        "processWaitType",
        "moveTrajectoryType",
        "robotInitType",
      ],
      default: [],
      isList: true,
      fullWidth: true,
    },
    compileFn: {
      default: COMPILE_FUNCTIONS.SIMPLE,
    },
    updateFields: {
      default: ["children"],
    }
  },
};

export const goalProgramType = merge(programFeatures, baseTypeData);