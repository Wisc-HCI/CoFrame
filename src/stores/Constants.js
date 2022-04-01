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
    BREAK: 4,
    PROCESS: 5,
    SKILL: 6,
    POSE: 7,
    ROBOT_MOTION: 8,
    GRIPPER_MOTION: 9,
    ROBOT_AGENT: 10,
    HUMAN_AGENT: 11,
    GRIPPER: 12,
    LINK: 13,
    PROPERTY: 14
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

export const DETAIL_TYPES = ['machineType', 'inputOutputType', 'processType', 'locationType', 'waypointType', 'thingType','fixtureType','toolType','robotAgentType','humanAgentType','gripperType']
export const TIMELINE_TYPES = ['programType', 'skillType', 'hierarchicalType', 
                        'moveGripperType', 'machineInitType', 'processStartType', 
                        'processWaitType', 'processStopType', 'moveTrajectoryType',
                        'robotInitType'
                    ]
export const REFERENCEABLE_OBJECTS = ["machineType", "fixtureType", "linkType", "toolType", "robotAgentType", "humanAgentType", 'gripperType'];