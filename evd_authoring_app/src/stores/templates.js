import { generateUuid } from "./generateUuid"

export const primitiveTemplates = {
    'node.primitive.delay.':{
        name: 'Delay',
        description: 'Delay robot for a specified time',
        editable: true,
        deleteable: true,
        parameters: {
            duration: 1
        }
    },
    'node.primitive.gripper.':{
        name: 'Gripper',
        description: 'Close or Open the robot gripper',
        editable: true,
        deleteable: true,
        parameters: {
            position: 0,
            speed: 1,
            effort: 1,
            semantic: 'releasing',
            thing_uuid: null
        }
    },
    'node.primitive.machine-primitive.machine-initialize.':{
        name: 'Machine Initialize',
        description: 'Initialize a machine',
        editable: true,
        deleteable: true,
        parameters: {
            machine_uuid: null
        }
    },
    'node.primitive.machine-primitive.machine-start.':{
        name: 'Machine Start',
        description: 'Start a machine',
        editable: true,
        deleteable: true,
        parameters: {
            machine_uuid: null
        }
    },
    'node.primitive.machine-primitive.machine-stop.':{
        name: 'Machine Stop',
        description: 'Stop a machine',
        editable: true,
        deleteable: true,
        parameters: {
            machine_uuid: null
        }
    },
    'node.primitive.machine-primitive.machine-wait.':{
        name: 'Machine Wait',
        description: 'Pause a machine',
        editable: true,
        deleteable: true,
        parameters: {
            machine_uuid: null
        }
    },
    'node.primitive.move-trajectory.':{
        name: 'Move Trajectory',
        description: 'Move the robot arm along a trajectory',
        editable: true,
        deleteable: true,
        parameters: {
            manual_safety: false,
            trajectory_uuid: null
        }
        
    },
    'node.primitive.move-unplanned.':{
        name: 'Move Unplanned',
        description: '[Dangerous] Move the robot to a specified location',
        editable: true,
        deleteable: true,
        parameters: {
            manual_safety: false,
            velocity: 1,
            move_type: 'joint',
            location_uuid: null
        }
    },
    'node.primitive.breakpoint':{
        name: 'Breakpoint',
        description: 'Stop execution at this point',
        editable: true,
        deleteable: true,
        parameters: {}
    },
    'node.primitive.skill-call.':{
        name: 'Skill Call',
        description: 'Execute a skill',
        editable: true,
        deleteable: true,
        parameters: {} // TODO: FILL IN
    }
}

export const containerTemplates = {
    'node.trajectory.':{
        name: 'New Trajectory',
        deleteable: true,
        editable: true,
        description: 'A movement by the robot from one location to another.', // could be ''

        start_location_uuid: null,
        end_location_uuid: null,
        waypoint_uuids: [],
        trace: null,
        velocity: 0.5, // suggestion (user configured)
        move_type: 'ee_ik' // or 'ee_ik'
    },
    'node.primitive.hierarchical.skill.':{
        name: 'New Macro',
        transform: {x:0,y:0},
        editable: true,
        deleteable: true,
        description: 'A reusable and configurable behavior that can be executed from the elsewhere in the program.',
        parameters: {},
        arguments: [],
        primitiveIds: []
    },
    'node.primitive.hierarchical.':{
        name: 'New Hierarchical',
        editable: true,
        deleteable: true,
        description: 'A simple grouping structure for related actions.',
        parameters: {},
        primitiveIds: []
    }
}

export const primitiveTypes = Object.keys(primitiveTemplates).filter(type=>(!type.includes('skill')));
export const containerTypes = Object.keys(containerTemplates);

export const fromPrimitiveTemplate = (type) => {
    return {uuid:generateUuid(type),type:type,parentData:{type:'drawer',uuid:null},...primitiveTemplates[type]}
}

export const fromContainerTemplate = (type) => {
    return {uuid:generateUuid(type),type:type,parentData:{type:'drawer',uuid:null},...containerTemplates[type]}
}
