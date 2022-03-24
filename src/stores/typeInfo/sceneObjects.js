import { EXTRA_TYPES, TYPES, SIMPLE_PROPERTY_TYPES } from "simple-vp";
import { MachineIconStyled, FixtureIconStyled, LinkIconStyled, ZoneIconStyled, ToolIconStyled } from "./icons";
import { FiMoreHorizontal } from "react-icons/fi";
import { merge } from 'lodash';
import { STATUS, COMPILE_FUNCTIONS } from "../Constants";
import { REFERENCEABLE_OBJECTS } from "../Constants";

const basicObject = {
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
            default: { x: 0, y: 0, z: 0 },
            isList: false,
            fullWidth: true
        },
        rotation: {
            name: 'Rotation',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: { w: 1, x: 0, y: 0, z: 0 },
            isList: false,
            fullWidth: true
        },
        relativeTo: {
            name: 'Relative To Object',
            accepts: REFERENCEABLE_OBJECTS,
            default: null,
            isList: false,
            nullValid: true
        },
        mesh: {
            name: 'Mesh',
            accepts: ["meshType"],
            default: null,
            isList: false,
            nullValid: true
        },
        status: {
            name: 'Status',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: STATUS.PENDING
        },
        compileFn: {
            name: 'Compile Function',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: COMPILE_FUNCTIONS.NULL
        },
        compiled: {
            name: 'Compiled',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: {}
        }
    }
}

const fixture = {
    name: 'Fixture',
    type: TYPES.OBJECT,
    instanceBlock: null,
    referenceBlock: {
        onCanvas: false,
        color: "#7f4658",
        icon: FixtureIconStyled,
        extras: []
    },
    properties: {
        showCollision: {
            name: 'Show Collision',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: false,
            isList: false,
            fullWidth: true
        },
        showName: {
            name: 'Show Name',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: false,
            isList: false,
            fullWidth: true
        },
        highlighted: {
            name: 'Highlighted',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: false,
            isList: false,
            fullWidth: true
        }
    }
}

const link = {
    name: 'Link',
    type: TYPES.OBJECT,
    instanceBlock: null,
    referenceBlock: {
        onCanvas: false,
        color: "#000000",
        icon: LinkIconStyled,
        extras: []
    },
    properties: {
        showCollision: {
            name: 'Show Collision',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: false,
            isList: false,
            fullWidth: true
        },
        showName: {
            name: 'Show Name',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: false,
            isList: false,
            fullWidth: true
        },
        highlighted: {
            name: 'Highlighted',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: false,
            isList: false,
            fullWidth: true
        },
        robot: {
            name: 'Robot Ref',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: '',
            isList: false,
            fullWidth: true
        },
        collision: {
            name: 'Collision',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: '',
            isList: false,
            fullWidth: true
        }
    }
}

const machine = {
    name: 'Machine',
    type: TYPES.OBJECT,
    instanceBlock: null,
    referenceBlock: {
        onCanvas: false,
        color: "#B3A533",
        icon: MachineIconStyled,
        extras: [
            EXTRA_TYPES.LOCKED_INDICATOR,
            {
                icon: FiMoreHorizontal,
                type: EXTRA_TYPES.DROPDOWN,
                contents: [
                    EXTRA_TYPES.DELETE_BUTTON,
                    EXTRA_TYPES.DEBUG_TOGGLE,
                    EXTRA_TYPES.SELECTION_TOGGLE
                ]
            }
        ]
    },
    properties: {
        collisionMesh: {
            name: 'Collision Mesh',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: "",
            isList: false,
            fullWidth: true
        }
    }
}

const zone = {
    name: 'zone',
    type: TYPES.OBJECT,
    instanceBlock: null,
    referenceBlock: {
        onCanvas: false,
        color: "#a56f83",
        icon: ZoneIconStyled,
        extras: []
    },
    properties: {
        agent: {
            name: 'Agent ID',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: "",
            isList: false,
            fullWidth: true
        },
        scale: {
            name: 'Scale',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: { x: null, y: null, z: null },
            isList: false,
            fullWidth: true
        }
    }
}

const tool = {
    name: 'Tool',
    type: TYPES.OBJECT,
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
                    EXTRA_TYPES.SELECTION_TOGGLE
                ]
            }
        ]
    },
    properties: {
        safe: {
            name: 'Safe',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: false,
            isList: false,
            fullWidth: true
        },
        weight: {
            name: 'Weight',
            type: SIMPLE_PROPERTY_TYPES.IGNORED,
            default: 0,
            isList: false,
            fullWidth: true
        },
        collisionMesh: {
            name: 'Collision Mesh',
            accepts: ["meshType"],
            default: null,
            isList: false,
            fullWidth: true
        }
    }
}


const sceneObjects = {
    machineType: merge(machine, basicObject),
    fixtureType: merge(fixture, basicObject),
    linkType: merge(link, basicObject),
    zoneType: merge(zone, basicObject),
    toolType: merge(tool, basicObject)
}

export default sceneObjects