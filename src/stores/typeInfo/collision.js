import { TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";
import { merge } from "lodash";

const baseCollision = {
  type: TYPES.OBJECT,
  instanceBlock: null,
  referenceBlock: null,
  properties: {
    status: {
      name: "Status",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: STATUS.PENDING,
    },
    compileFn: {
      name: "Compile Function",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: COMPILE_FUNCTIONS.NULL,
    },
    compiled: {
      name: "Compiled",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: {},
    },
    singleton: {
      name: "singleton",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: true,
    },
  },
};

const collisionShape = {
  name: "Collision Shape",
  properties: {
    position: {
      name: "Local Position",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: { x: 0, y: 0, z: 0 },
      isList: false,
      fullWidth: true,
    },
    rotation: {
      name: "Local Rotation",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: { w: 0, x: 0, y: 0, z: 0 },
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
    keyword: {
      name: "Robot Scene Keyword",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: "",
      isList: false,
      fullWidth: true,
    },
    radius: {
      name: "Radius",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: 0,
      isList: false,
      fullWidth: true,
    },
    length: {
      name: "Length",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: 0,
      isList: false,
      fullWidth: true,
    },
    extraParams: {
      name: "Simple Shape Paramaters",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: null,
      isList: false,
      fullWidth: true,
    },
    updateFields: {
      name: "Update Fields",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: ["position", "rotation", "scale"],
    },
  },
};

const collisionBody = {
  name: "Collision Body",
  properties: {
    componentShapes: {
      name: "Collision Shapes",
      accepts: ["collisionShapeType"],
      default: [],
      isList: true,
      fullWidth: true,
    },
    updateFields: {
      name: "Update Fields",
      type: SIMPLE_PROPERTY_TYPES.IGNORED,
      default: ["componentShapes"],
    },
  },
};

const collisionTypes = {
  collisionBodyType: merge(collisionBody, baseCollision),
  collisionShapeType: merge(collisionShape, baseCollision),
};

export default collisionTypes;
