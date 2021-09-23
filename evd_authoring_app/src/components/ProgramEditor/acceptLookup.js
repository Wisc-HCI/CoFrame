
export const acceptLookup = {
    'drawer':{
        default:{
            accepts:[],
            placement:'sequence'
        }
    },
    'grid': {
        primitiveIds:{
            accepts:[
                'node.primitive.hierarchical.skill.',
                'node.primitive.hierarchical.program.'
            ],
            placement:'free'
        }
    },
    'trash': {
        all:{
            accepts:[
                'uuid-machine',
                'uuid-waypoint',
                'node.primitive.hierarchical.skill.',
                'node.primitive.delay.',
                'node.primitive.breakpoint',
                'node.primitive.gripper.',
                'node.primitive.machine-primitive.machine-initialize.',
                'node.primitive.machine-primitive.machine-start.',
                'node.primitive.machine-primitive.machine-stop.',
                'node.primitive.machine-primitive.machine-wait.',
                'node.primitive.move-trajectory.',
                'node.primitive.move-unplanned.',
                'node.primitive.skill-call.',
                'node.primitive.hierarchical.',
                'node.trajectory.',
                'uuid-location',
                'uuid-waypoint',
                'uuid-thing',
                'uuid-trajectory'
            ],
            placement:'single'
        }
    },
    'node.primitive.hierarchical.program.': {
        primitiveIds:{
            accepts:[
                'node.primitive.delay.',
                'node.primitive.breakpoint',
                'node.primitive.gripper.',
                'node.primitive.machine-primitive.machine-initialize.',
                'node.primitive.machine-primitive.machine-start.',
                'node.primitive.machine-primitive.machine-stop.',
                'node.primitive.machine-primitive.machine-wait.',
                'node.primitive.move-trajectory.',
                'node.primitive.move-unplanned.',
                'node.primitive.skill-call.',
                'node.primitive.hierarchical.'
            ],
            placement:'sequence'
        }
    },
    'node.primitive.hierarchical.skill.': {
        primitiveIds:{
            accepts:[
                'node.primitive.delay.',
                'node.primitive.breakpoint',
                'node.primitive.gripper.',
                'node.primitive.machine-primitive.machine-initialize.',
                'node.primitive.machine-primitive.machine-start.',
                'node.primitive.machine-primitive.machine-stop.',
                'node.primitive.machine-primitive.machine-wait.',
                'node.primitive.move-trajectory.',
                'node.primitive.move-unplanned.',
                'node.primitive.skill-call.',
                'node.primitive.hierarchical.'
            ],
            placement:'sequence'
        }
    },
    'node.primitive.hierarchical.': {
        primitiveIds:{
            accepts:[
                'node.primitive.delay.',
                'node.primitive.breakpoint',
                'node.primitive.gripper.',
                'node.primitive.machine-primitive.machine-initialize.',
                'node.primitive.machine-primitive.machine-start.',
                'node.primitive.machine-primitive.machine-stop.',
                'node.primitive.machine-primitive.machine-wait.',
                'node.primitive.move-trajectory.',
                'node.primitive.move-unplanned.',
                'node.primitive.skill-call.',
                'node.primitive.hierarchical.'
            ],
            placement:'sequence'
        }
    },
    'node.primitive.delay.':{},
    'node.primitive.breakpoint':{},
    'node.primitive.gripper.':{},
    'node.primitive.machine-primitive.machine-initialize.':{},
    'node.primitive.machine-primitive.machine-start.':{
        machine_uuid: {
            accepts:['uuid-machine'],
            placment: 'single'
        }
    },
    'node.primitive.machine-primitive.machine-stop.':{},
    'node.primitive.machine-primitive.machine-wait.':{},
    'node.primitive.move-trajectory.':{},
    'node.primitive.move-unplanned.':{},
    'node.primitive.skill-call.':{},
    'node.trajectory.':{
        start_location_uuid:{
            accepts:['node.pose.waypoint.location.'],
            placement: 'single'
        },
        end_location_uuid:{
            accepts:['node.pose.waypoint.location.'],
            placement: 'single'
        },
        waypoint_uuids:{
            accepts:['node.pose.waypoint.'],
            placement: 'sequence'
        }
    }
}