import { generateUuid } from "./generateUuid"

export const primitiveTemplates = {
    'delay': {
        name: 'Delay',
        description: 'Delay robot for a specified time',
        editable: true,
        deleteable: true,
        parameters: {
            duration: 1
        }
    },
    'gripper': {
        name: 'Move Gripper',
        description: 'Close or Open the robot gripper',
        editable: true,
        deleteable: true,
        parameters: {
            position: 50,
            speed: 50,
            effort: 1,
            semantic: 'releasing',
            thing: null
        }
    },
    'machine-initialize': {
        name: 'Machine Initialize',
        description: 'Initialize a machine',
        editable: true,
        deleteable: true,
        parameters: {
            machine: null
        }
    },
    'process-start': {
        name: 'Process Start',
        description: 'Start a process',
        editable: true,
        deleteable: true,
        parameters: {
            process: null,
            recipe: null
        }
    },
    'process-stop': {
        name: 'Process Complete',
        description: 'Complete a process',
        editable: true,
        deleteable: true,
        parameters: {
            process: null,
            recipe: null
        }
    },
    'process-wait': {
        name: 'Process Wait',
        description: 'Pause a process',
        editable: true,
        deleteable: true,
        parameters: {
            process: null,
            recipe: null
        }
    },
    'move-trajectory': {
        name: 'Move Trajectory',
        description: 'Move the robot arm along a trajectory',
        editable: true,
        deleteable: true,
        parameters: {
            velocity: 0.5, // suggestion (user configured)
            moveType: 'ee_ik',
            trajectory: null
        }

    },
    'move-unplanned': {
        name: 'Move Unplanned',
        description: '[Dangerous] Move the robot to a specified location',
        editable: true,
        deleteable: true,
        parameters: {
            velocity: 1,
            moveType: 'joint',
            location: null
        }
    },
    'breakpoint': {
        name: 'Breakpoint',
        description: 'Stop execution at this point',
        editable: true,
        deleteable: true,
        parameters: {}
    },
    'skill-call': {
        name: 'Skill Call',
        description: 'Execute a skill',
        editable: true,
        deleteable: true,
        parameters: {} // TODO: FILL IN
    }
}

export const containerTemplates = {
    'trajectory': {
        name: 'New Trajectory',
        deleteable: true,
        editable: true,
        description: 'A movement by the robot from one location to another.', // could be ''
        startLocation: null,
        endLocation: null,
        waypoints: [],
        trace: null
    },
    'skill': {
        name: 'New Skill',
        transform: { x: 0, y: 0 },
        editable: true,
        deleteable: true,
        description: 'A reusable and configurable behavior that can be executed from the elsewhere in the program.',
        arguments: [],
        children: []
    },
    'hierarchical': {
        name: 'New Group',
        editable: true,
        deleteable: true,
        description: 'A simple grouping structure for related actions.',
        children: [],
        transform: {x: 0, y: 0}
    },
    'program': {
        name: 'New Program',
        editable: true,
        deleteable: false,
        description: 'The top-level program',
        parameters: {},
        children: [],
        transform: {x: 0, y: 0}
    }
}

export const createWaypoint = () => ({
    type: 'waypoint',
    uuid: generateUuid('waypoint'),
    name: `New Waypoint`,
    deleteable: true,
    editable: true,
    description: 'Some descriptor string (optional)',
    reachable: false,
    trace: null,
    position: {
        x: .5 * (Math.random() - 0.5),
        y: .5 * (Math.random() - 0.5),
        z: .5 * (Math.random() - 0.5) + 0.25
    },
    orientation: {
        x: 0,
        y: 0,
        z: 0,
        w: 1
    }
}
)

export const createLocation = () => ({
    type: 'location',
    uuid: generateUuid('location'),
    name: `New Location`,
    deleteable: true,
    editable: true,
    description: 'Some descriptor string (optional)',
    reachable: false,
    trace: null,
    position: {
        x: .5 * (Math.random() - 0.5),
        y: .5 * (Math.random() - 0.5),
        z: .5 * (Math.random() - 0.5) + 0.25
    },
    orientation: {
        x: 0,
        y: 0,
        z: 0,
        w: 1
    }
})

export const createThing = (thingType) => ({
    type: 'thing',
    uuid: generateUuid('thing'),
    name: `New Thing`,
    deleteable: true,
    editable: true,
    description: 'Some descriptor string',
    thingType
})

export const createThingType = () => ({
    type: 'thingType',
    uuid: generateUuid('thingType'),
    name: `New Thing Type`,
    deleteable: true,
    editable: true,
    description: 'Some descriptor string',
    mesh: 'box',
    safe: true,
    weight: 1 // kg
})

export const createMachine = () => ({
    type: 'machine',
    uuid: generateUuid('machine'),
    name: `New Machine`,
    deleteable: true,
    editable: true,
    description: 'Some descriptor string',
    mesh: 'box',
    link: "world",
    position: {
        x: .5 * (Math.random() - 0.5),
        y: .5 * (Math.random() - 0.5),
        z: .5 * (Math.random() - 0.5) + 0.25
    },
    orientation: {
        x: 0,
        y: 0,
        z: 0,
        w: 1
    }
})

export const createProcess = () => ({
    type: 'process',
    uuid: generateUuid('process'),
    name: `New Process`,
    deleteable: true,
    editable: true,
    description: 'Some descriptor string',
    processTime: 0,
    machine: null,
    inputs: [],
    outputs: []
})

export const createRobotAgent = () => ({
    type: 'robot-agent',
    uuid: generateUuid('robot-agent'),
    name: 'New Robot Agent',
    deleteable: true,
    editable: true,
    description: 'Some descriptor string'
})

export const createHumanAgent = () => ({
    type: 'human-agent',
    uuid: generateUuid('human-agent'),
    name: 'New Human Agent',
    deleteable: true,
    editable: true,
    description: 'Some descriptor string'
})

export const createZone = (agent) => ({
    type: 'zone',
    uuid: generateUuid('zone'),
    name: `New Zone`,
    deleteable: true,
    editable: true,
    description: 'Some descriptor string',
    agent,
    scale: {
        x: .5 * (Math.random() - 0.5),
        y: .5 * (Math.random() - 0.5),
        z: .5 * (Math.random() - 0.5) + 0.25
    },
    position: {
        x: .5 * (Math.random() - 0.5),
        y: .5 * (Math.random() - 0.5),
        z: .5 * (Math.random() - 0.5) + 0.25
    },
    orientation: {
        x: 0,
        y: 0,
        z: 0,
        w: 1
    }
})

/*
Inputs/Outputs have the following format:
{
    thingType: str,
    position: {x: number, y: number, z: number},
    orientation: {w: number, x: number, y: number, z: number}
}
*/

export const primitiveTypes = Object.keys(primitiveTemplates).filter(type => (!type.includes('skill')));
export const containerTypes = Object.keys(containerTemplates);

export const fromPrimitiveTemplate = (type) => {
    return { uuid: generateUuid(type), type, ...primitiveTemplates[type] }
}

export const fromContainerTemplate = (type) => {
    return { uuid: generateUuid(type), type, ...containerTemplates[type] }
}