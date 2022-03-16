export const STATUS = {
    PENDING: 0,
    VALID: 1,
    FAILED: 2
}

export const STEP_CALCULATOR = {
    NULL: 0,
    SIMPLE: 1,
    MACHINE: 2,
    DELAY: 3,
    PROCESS: 4,
    SKILL: 5,
    POSE: 6,
    GRIPPER: 7,
    ROBOT_MOTION: 8,
    BREAK: 9,
    AGENT: 10
}

export const STEP_TYPE = {
    LANDMARK: 0,
    SCENE_UPDATE: 1,
    SCENE_REMOVE: 2,
    RAW_DATA: 3,
    ACTION_START: 4,
    ACTION_END: 5,
    PROCESS_START: 6,
    PROCESS_END: 7
}

export const ROOT_BOUNDS = [
    {value:0.0,delta:0.0},{value:0.0,delta:0.0},{value:0.0,delta:0.0}, // Translational
    {value:0.0,delta:0.0},{value:0.0,delta:0.0},{value:0.0,delta:0.0}  // Rotational
]

export const DETAIL_TYPES = ['machineType', 'inputOutputType', 'processType', 'locationType', 'waypointType', 'thingType','fixtureType']
export const TIMELINE_TYPES = ['programType', 'skillType', 'hierarchicalType', 
                        'gripperType', 'machineInitType', 'processStartType', 
                        'processWaitType', 'processStopType', 'moveTrajectoryType',
                        'robotInitType'
                    ]
