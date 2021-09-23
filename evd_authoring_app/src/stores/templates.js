import { generateUuid } from "./generateUuid"
import { COLLISION_MESHES } from "./initialSim"

export const primitiveTemplates = {
    'node.primitive.delay.': {
        name: 'Delay',
        description: 'Delay robot for a specified time',
        editable: true,
        deleteable: true,
        parameters: {
            duration: 1
        }
    },
    'node.primitive.gripper.': {
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
    'node.primitive.machine-primitive.machine-initialize.': {
        name: 'Machine Initialize',
        description: 'Initialize a machine',
        editable: true,
        deleteable: true,
        parameters: {
            machine_uuid: null
        }
    },
    'node.primitive.machine-primitive.machine-start.': {
        name: 'Machine Start',
        description: 'Start a machine',
        editable: true,
        deleteable: true,
        parameters: {
            machine_uuid: null
        }
    },
    'node.primitive.machine-primitive.machine-stop.': {
        name: 'Machine Stop',
        description: 'Stop a machine',
        editable: true,
        deleteable: true,
        parameters: {
            machine_uuid: null
        }
    },
    'node.primitive.machine-primitive.machine-wait.': {
        name: 'Machine Wait',
        description: 'Pause a machine',
        editable: true,
        deleteable: true,
        parameters: {
            machine_uuid: null
        }
    },
    'node.primitive.move-trajectory.': {
        name: 'Move Trajectory',
        description: 'Move the robot arm along a trajectory',
        editable: true,
        deleteable: true,
        parameters: {
            manual_safety: false,
            trajectory_uuid: null
        }

    },
    'node.primitive.move-unplanned.': {
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
    'node.primitive.breakpoint': {
        name: 'Breakpoint',
        description: 'Stop execution at this point',
        editable: true,
        deleteable: true,
        parameters: {}
    },
    'node.primitive.skill-call.': {
        name: 'Skill Call',
        description: 'Execute a skill',
        editable: true,
        deleteable: true,
        parameters: {} // TODO: FILL IN
    }
}

export const containerTemplates = {
    'node.trajectory.': {
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
    'node.primitive.hierarchical.skill.': {
        name: 'New Skill',
        transform: { x: 0, y: 0 },
        editable: true,
        deleteable: true,
        description: 'A reusable and configurable behavior that can be executed from the elsewhere in the program.',
        parameters: {},
        arguments: [],
        primitiveIds: []
    },
    'node.primitive.hierarchical.': {
        name: 'New Group',
        editable: true,
        deleteable: true,
        description: 'A simple grouping structure for related actions.',
        parameters: {},
        primitiveIds: []
    }
}

export const createWaypoint = () => ({
    type: 'node.pose.waypoint.',
    uuid: generateUuid('node.pose.waypoint.'),
    name: `New Waypoint`,
    deleteable: true,
    editable: true,
    description: 'Some descriptor string (optional)', // could be ''

    link: 'world', //'' means default to world or app
    joints: {
        type: 'node.joints.',
        uuid: generateUuid('node.joints.'),
        name: '',
        deleteable: false,
        editable: true,
        description: '', // could be ''

        joint_positions: null,
        joint_names: null,
        reachable: false, // better than having to check the array to generate the flag
        length: 5 // this is enforced on the backend for positions and names
    },
    position: {
        type: 'node.position.',
        uuid: generateUuid('node.position.'),
        name: '',
        deleteable: false,
        editable: true,
        description: '', // could be ''

        x: .5 * (Math.random() - 0.5),
        y: .5 * (Math.random() - 0.5),
        z: .5 * (Math.random() - 0.5) + 0.25
    },
    orientation: {
        type: 'node.orientation.',
        uuid: generateUuid('node.orientation.'),
        name: '',
        deleteable: false,
        editable: true,
        description: '', // could be ''

        x: 0,
        y: 0,
        z: 0,
        w: 1
    }
}
)

export const createLocation = () => ({
    type: 'node.pose.waypoint.location.',
    uuid: generateUuid('node.pose.waypoint.location'),
    name: `New Location`,
    deleteable: true,
    editable: true,
    description: 'Some descriptor string (optional)', // could be ''

    link: '', //'' means default to world or app
    joints: {
        type: 'node.joints.',
        uuid: generateUuid('node.joints.'),
        name: '',
        deleteable: false,
        editable: true,
        description: '', // could be ''
        joint_positions: null,
        joint_names: null,
        reachable: false, // better than having to check the array to generate the flag
        length: 5 // this is enforced on the backend for positions and names
    },
    position: {
        type: 'node.position.',
        uuid: generateUuid('node.position.'),
        name: '',
        deleteable: false,
        editable: true,
        description: '', // could be ''
        x: .5 * (Math.random() - 0.5),
        y: .5 * (Math.random() - 0.5),
        z: .5 * (Math.random() - 0.5) + 0.25
    },
    orientation: {
        type: 'node.orientation.',
        uuid: generateUuid('node.orientation.'),
        name: '',
        deleteable: false,
        editable: true,
        description: '', // could be ''
        x: 0,
        y: 0,
        z: 0,
        w: 1
    }
})

export const createPlaceholder = (thingTypeUuid) => ({
    type: 'node.placeholder.',
    uuid: generateUuid('node.placeholder.'),
    name: `New Placeholder`,
    deleteable: true,
    editable: true,
    description: 'Some descriptor string',

    pending_node: {
        type: 'node.pose.thing.',
        uuid: generateUuid('node.pose.thing.'),
        name: `New Thing`,
        deleteable: true,
        editable: true,
        description: 'Some descriptor string (optional)', // could be ''

        thing_type_uuid: thingTypeUuid, // gets a thing_type from other fake data
        position: '<pending>',
        orientation: '<pending>'
    },
    pending_fields: [
        'position',
        'orientation'
    ]
})

export const createMachine = (mesh) => ({
    type: 'node.machine.',
    uuid: generateUuid('node.machine.'),
    name: `New Machine`,
    deleteable: true,
    editable: true,
    description: 'Some descriptor string (optional)', // could be ''

    process_time: 5, //sec
    inputs: {}

    , // make this a tuple as value of keys

    outputs: {},
    mesh_id: mesh,
    pose_offset: { // Local transform
        type: 'node.pose.',
        uuid: generateUuid('node.pose.'),
        name: `Pose-0`,
        deleteable: false,
        editable: false,
        description: '', // could be ''

        link: '', //'' means default to world or app
        position: {
            type: 'node.position.',
            uuid: generateUuid('node.position.'),
            name: `Position-0`,
            deleteable: false,
            editable: false,
            description: '', // could be ''

            x: 0,
            y: 0,
            z: 0
        },
        orientation: {
            type: 'node.orientation.',
            uuid: generateUuid('node.orientation.'),
            name: `Orientation-0`,
            deleteable: false,
            editable: false,
            description: '', // could be ''

            x: 0,
            y: 0,
            z: 0,
            w: 1
        }
    },
    link: 'world',
    collision_mesh_uuid: COLLISION_MESHES[mesh]
})

export const primitiveTypes = Object.keys(primitiveTemplates).filter(type => (!type.includes('skill')));
export const containerTypes = Object.keys(containerTemplates);

export const fromPrimitiveTemplate = (type) => {
    return { uuid: generateUuid(type), type, ...primitiveTemplates[type] }
}

export const fromContainerTemplate = (type) => {
    return { uuid: generateUuid(type), type, ...containerTemplates[type] }
}