import { TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { merge } from "lodash";
import { baseTypeData } from "./baseType";
import { COMPILE_FUNCTIONS } from "../Constants";

const collisionShapeDoc = 'A data object containing size, position, and shape information of the collider';
const collisionBodyDoc = 'A collection of [Collision Shapes](collisionShapeType) attached to some scene object';

const baseCollision = {
  type: TYPES.OBJECT,
  instanceBlock: null,
  referenceBlock: null,
  properties: {
    compileFn: {
      default: COMPILE_FUNCTIONS.NULL,
    },
    singleton: {
      default: true,
    },
  },
};

const collisionShape = {
  name: "Collision Shape",
  description: collisionShapeDoc,
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
      default: { x: 0, y: 0, z: 0 },
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
    // radius: {
    //   name: "Radius",
    //   type: SIMPLE_PROPERTY_TYPES.IGNORED,
    //   default: 0,
    //   isList: false,
    //   fullWidth: true,
    // },
    // length: {
    //   name: "Length",
    //   type: SIMPLE_PROPERTY_TYPES.IGNORED,
    //   default: 0,
    //   isList: false,
    //   fullWidth: true,
    // },
    extraParams: {
      name: "Simple Shape Parameters",
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
  description: collisionBodyDoc,
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
  collisionBodyType: merge(collisionBody, baseCollision, baseTypeData),
  collisionShapeType: merge(collisionShape, baseCollision, baseTypeData),
};

export default collisionTypes;
