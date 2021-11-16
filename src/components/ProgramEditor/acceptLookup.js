
export const acceptLookup = {
    'drawer':{
        default:{
            accepts:[],
            placement:'sequence'
        }
    },
    'grid': {
        children:{
            accepts:[
                'skill',
                'program'
            ],
            placement:'free'
        }
    },
    'trash': {
        all:{
            accepts:[
                'uuid-machine',
                'uuid-waypoint',
                'skill',
                'delay',
                'breakpoint',
                'gripper',
                'machine-initialize',
                'process-start',
                'process-stop',
                'process-wait',
                'move-trajectory',
                'move-unplanned',
                'skill-call',
                'hierarchical',
                'trajectory',
                'uuid-location',
                'uuid-waypoint',
                'uuid-thing',
                'uuid-trajectory'
            ],
            placement:'single'
        }
    },
    'program': {
        children:{
            accepts:[
                'delay',
                'breakpoint',
                'gripper',
                'machine-initialize',
                'process-start',
                'process-stop',
                'process-wait',
                'move-trajectory',
                'move-unplanned',
                'skill-call',
                'hierarchical'
            ],
            placement:'sequence'
        }
    },
    'skill': {
        children:{
            accepts:[
                'delay',
                'breakpoint',
                'gripper',
                'machine-initialize',
                'process-start',
                'process-stop',
                'process-wait',
                'move-trajectory',
                'move-unplanned',
                'skill-call',
                'hierarchical'
            ],
            placement:'sequence'
        }
    },
    'hierarchical': {
        children:{
            accepts:[
                'delay',
                'breakpoint',
                'gripper',
                'machine-initialize',
                'process-start',
                'process-stop',
                'process-wait',
                'move-trajectory',
                'move-unplanned',
                'skill-call',
                'hierarchical'
            ],
            placement:'sequence'
        }
    },
    'delay':{},
    'breakpoint':{},
    'gripper':{},
    'machine-initialize':{},
    'process-start':{
        machine_uuid: {
            accepts:['uuid-machine'],
            placment: 'single'
        }
    },
    'process-stop':{},
    'process-wait':{},
    'move-trajectory':{},
    'move-unplanned':{},
    'skill-call':{},
    'node.trajectory':{
        startLocation:{
            accepts:['location'],
            placement: 'single'
        },
        endLocation:{
            accepts:['location'],
            placement: 'single'
        },
        waypoints:{
            accepts:['waypoint'],
            placement: 'sequence'
        }
    }
}