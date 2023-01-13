import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import {
  MachineIconStyled,
  FixtureIconStyled,
  LinkIconStyled,
  ZoneIconStyled,
  ToolIconStyled,
} from "./icons";
import { FiMoreHorizontal } from "react-icons/fi";
import { merge } from "lodash";
import { COMPILE_FUNCTIONS } from "../Constants";
import { REFERENCEABLE_OBJECTS } from "../Constants";
import { baseTypeData } from "./baseType";

const fixtureDoc = `Fixtures are static objects related to other components in the scene. For example, one could put a [Machine](machineType) or [Tools](toolType) on the surface of a table-like fixture. While the [Robot Agent](robotAgentType) cannot directly interact with fixtures, they can nevertheless collide.`;
const linkDoc = `Links are parts of the [Robot](robotAgentType), connected by joints. Each link can define collision body, which can be toggled on and off in the Simulator Area`;
const machineDoc = `Machines are static fixtures in the environment that are able to perform create, consume, or modify [Things](thingType) in the [Program](programType) through the use of [Processes](processType). They define specific [Regions](zoneType) that are used for depositing or retrieving these things.`;
const zoneDoc = `Zones are areas designated to be primarily occupied by a single agent (either [Robot](robotAgentType) or [Human](humanAgentType)). While multiple agents can co-exist in a given zone, ideally this is minimized.`;
const toolDoc = `Like [Machines](machineType), tools are able to create, consume, or modify [Things](thingType) in the [Program](programType) through the use of [Processes](processType). Unlike machines, however, tools can be moved or temporarily used as inputs and outputs in processes.`;

const basicObject = {
  properties: {
    position: {
      name: "Position",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: { x: 0, y: 0, z: 0 },
      isList: false,
      fullWidth: true,
    },
    rotation: {
      name: "Rotation",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: { w: 1, x: 0, y: 0, z: 0 },
      isList: false,
      fullWidth: true,
    },
    relativeTo: {
      name: "Relative To Object",
      accepts: REFERENCEABLE_OBJECTS,
      default: null,
      isList: false,
      nullValid: true,
    },
    mesh: {
      name: "Mesh",
      accepts: ["meshType"],
      default: null,
      isList: false,
      nullValid: true,
    },
    collision: {
      name: "Collision",
      accepts: ["collisionBodyType"],
      default: null,
      isList: false,
      fullWidth: true,
    },
    singleton: {
      default: true,
    },
  },
};

const fixture = {
  name: "Fixture",
  type: TYPES.OBJECT,
  description: fixtureDoc,
  instanceBlock: null,
  referenceBlock: {
    onCanvas: false,
    color: "#7f4658",
    icon: FixtureIconStyled,
    extras: [],
  },
  properties: {
    compileFn: { default: COMPILE_FUNCTIONS.PROPERTY },
    updateFields: { default: ["position", "rotation", "relativeTo"] },
  },
};

const link = {
  name: "Link",
  type: TYPES.OBJECT,
  description: linkDoc,
  instanceBlock: null,
  referenceBlock: {
    onCanvas: false,
    color: "#000000",
    icon: LinkIconStyled,
    extras: [],
  },
  properties: {
    agent: {
      name: "Agent",
      accepts: ["robotAgentType", "gripperType"],
      default: null,
      isList: false,
    },
    frameKey: {
      name: "Frame Key",
      type: SIMPLE_PROPERTY_TYPES.STRING,
      default: "",
      isList: false
    },
    compileFn: { default: COMPILE_FUNCTIONS.LINK },
    updateFields: { default: ["position", "rotation", "relativeTo", "agent","frameKey"] },
  },
};

const machine = {
  name: "Machine",
  type: TYPES.OBJECT,
  description: machineDoc,
  instanceBlock: null,
  referenceBlock: {
    onCanvas: false,
    color: "#B3A533",
    icon: MachineIconStyled,
    extras: [
      EXTRA_TYPES.LOCKED_INDICATOR,
      EXTRA_TYPES.NAME_EDIT_TOGGLE,
      {
        icon: FiMoreHorizontal,
        type: EXTRA_TYPES.DROPDOWN,
        contents: [
          EXTRA_TYPES.DELETE_BUTTON,
          EXTRA_TYPES.DOC_TOGGLE,
          EXTRA_TYPES.SELECTION_TOGGLE,
        ],
      },
    ],
  },
  properties: {
    compileFn: { default: COMPILE_FUNCTIONS.PROPERTY },
    updateFields: { default: ["position", "rotation", "relativeTo"] },
  },
};

const zone = {
  name: "zone",
  type: TYPES.OBJECT,
  description: zoneDoc,
  instanceBlock: null,
  referenceBlock: {
    onCanvas: false,
    color: "#a56f83",
    icon: ZoneIconStyled,
    extras: [],
  },
  properties: {
    agent: {
      name: "Agent ID",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: "",
      isList: false,
      fullWidth: true,
    },
    scale: {
      name: "Scale",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: { x: 1, y: 1, z: 1 },
      isList: false,
      fullWidth: true,
    },
    compileFn: { default: COMPILE_FUNCTIONS.PROPERTY },
    updateFields: {
      default: ["position", "rotation", "relativeTo", "scale", "agent"],
    },
  },
};

const tool = {
  name: "Tool",
  type: TYPES.OBJECT,
  description: toolDoc,
  instanceBlock: null,
  referenceBlock: {
    onCanvas: false,
    color: "#c68a2f",
    icon: ToolIconStyled,
    extras: [
      EXTRA_TYPES.LOCKED_INDICATOR,
      {
        icon: FiMoreHorizontal,
        type: EXTRA_TYPES.DROPDOWN,
        contents: [
          EXTRA_TYPES.DELETE_BUTTON,
          EXTRA_TYPES.DEBUG_TOGGLE,
          EXTRA_TYPES.SELECTION_TOGGLE,
        ],
      },
    ],
  },
  properties: {
    graspPoints: {
      name: 'Grasp Points',
      accepts: ['graspPointType'],
      default: [],
      isList: true
    },
    safe: {
      name: "Safe",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: false,
      isList: false,
      fullWidth: true,
    },
    weight: {
      name: "Weight",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: 0,
      isList: false,
      fullWidth: true,
    },
    compileFn: { default: COMPILE_FUNCTIONS.PROPERTY },
    updateFields: { default: ['graspPoints', 'safe', 'weight', 'position', 'rotation']}
  },
};

const sceneObjects = {
  machineType: merge(machine, basicObject, baseTypeData),
  fixtureType: merge(fixture, basicObject, baseTypeData),
  linkType: merge(link, basicObject, baseTypeData),
  zoneType: merge(zone, basicObject, baseTypeData),
  toolType: merge(tool, basicObject, baseTypeData),
};

export default sceneObjects;
