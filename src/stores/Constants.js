export const STATUS = {
    PENDING: 0,
    VALID: 1,
    FAILED: 2
}

export const COMPILE_FUNCTIONS = {
    NULL: 0,
    SIMPLE: 1,
    MACHINE: 2,
    DELAY: 3,
    PROCESS: 4,
    SKILL: 5,
    POSE: 6,
    GRIPPER_MOTION: 7,
    ROBOT_MOTION: 8,
    BREAK: 9,
    AGENT: 10,
    GRIPPER: 11,
    LINK: 12,
    PROPERTY: 13
}

export const STEP_TYPE = {
    LANDMARK: 0,
    SCENE_UPDATE: 1,
    RAW_DATA: 2,
    ACTION_START: 3,
    ACTION_END: 4,
    PROCESS_START: 5,
    PROCESS_END: 6
}

export const ROOT_BOUNDS = [
    {value:0.0,delta:0.0},{value:0.0,delta:0.0},{value:0.0,delta:0.0}, // Translational
    {value:0.0,delta:0.0},{value:0.0,delta:0.0},{value:0.0,delta:0.0}  // Rotational
]

export const DETAIL_TYPES = ['machineType', 'inputOutputType', 'processType', 'locationType', 'waypointType', 'thingType','fixtureType','toolType']
export const TIMELINE_TYPES = ['programType', 'skillType', 'hierarchicalType', 
                        'gripperType', 'machineInitType', 'processStartType', 
                        'processWaitType', 'processStopType', 'moveTrajectoryType',
                        'robotInitType'
                    ]
export const REFERENCEABLE_OBJECTS = ["machineType", "fixtureType", "linkType", "toolType", "robotAgentType", "humanAgentType"];