import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { SkillIconStyled } from "./icons";
import {
  FiMoreHorizontal,
  FiAlertOctagon,
  FiThumbsUp,
  FiAlertTriangle,
  FiRefreshCw,
} from "react-icons/fi";
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";

export const skillType = {
  name: "Skill",
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
          EXTRA_TYPES.NAME_EDIT_TOGGLE,
          EXTRA_TYPES.SELECTION_TOGGLE,
          EXTRA_TYPES.DELETE_BUTTON,
          EXTRA_TYPES.LOCKED_INDICATOR,
          EXTRA_TYPES.DEBUG_TOGGLE,
          {
            type: EXTRA_TYPES.ADD_ARGUMENT_GROUP,
            allowed: [
              "machineType",
              "locationType",
              "thingType",
              "toolType",
              "trajectoryType",
            ],
          },
        ],
      },
    ],
  },
  callBlock: {
    onCanvas: false,
    color: "#62869e",
    icon: SkillIconStyled,
    extras: [
      EXTRA_TYPES.LOCKED_INDICATOR,
      {
        icon: FiMoreHorizontal,
        type: EXTRA_TYPES.DROPDOWN,
        contents: [
          {
            type: EXTRA_TYPES.INDICATOR_ICON,
            accessor: (data) => {
              if (data.properties.status === STATUS.FAILED) {
                return <FiAlertOctagon color="white" fill="red" />;
              } else if (data.properties.status === STATUS.VALID) {
                return <FiThumbsUp color="white" />;
              } else if (data.properties.status === STATUS.WARN) {
                return <FiAlertTriangle color="white" fill="#ff7300" />;
              } else if (data.properties.status === STATUS.PENDING) {
                return <FiRefreshCw className="rotate" />;
              } else {
                return <FiRefreshCw className="rotate" />;
              }
            },
            label: "Status",
          },
          EXTRA_TYPES.SELECTION_TOGGLE,
          EXTRA_TYPES.DELETE_BUTTON,
        ],
      },
    ],
  },
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
        "moveUnplannedType",
        "robotInitType",
      ],
      default: [],
      isList: true,
      fullWidth: true,
    },
    status: {
      name: "Status",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: STATUS.PENDING,
    },
    compileFn: {
      name: "Compile Function",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: COMPILE_FUNCTIONS.SIMPLE,
    },
    compiled: {
      name: "Compiled",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: {},
    },
    updateFields: {
      name: "Update Fields",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: ["children"],
    },
    singleton: {
      name: "singleton",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: false,
    },
  },
};
