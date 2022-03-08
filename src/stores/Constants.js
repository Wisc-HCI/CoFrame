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
    BREAK: 9
}

export const STEP_TYPE = {
    LANDMARK: 0,
    SCENE_UPDATE: 1,
    SCENE_REMOVE: 2,
    RAW_DATA: 3
}

export const ROOT_BOUNDS = [
    {value:0.0,delta:0.0},{value:0.0,delta:0.0},{value:0.0,delta:0.0}, // Translational
    {value:0.0,delta:0.0},{value:0.0,delta:0.0},{value:0.0,delta:0.0}  // Rotational
]