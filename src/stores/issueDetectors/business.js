import { STATUS, STEP_TYPE } from "../Constants";
import { generateUuid } from "../generateUuid"
import {idleTimeEstimate, durationEstimate} from "../helpers"

// Requires trace timing + delay and machine wait primitives (start computation after move_unplanned in initialize)
export const findCycleTimeIssues = ({program, stats}) => {
    let issues = {};

    if (!program || program.properties.status !== STATUS.VALID) {
        return [issues, {}];
    }

    const estimate = durationEstimate(program.properties.compiled["{}"].steps);

    // get prior values
    let priorData = [];
    let i = 0;
    for (i = 0; i < stats.length; i++) {
        if (stats[i].cycleTime) {
            priorData.push({x:i, cycleTime:stats[i].cycleTime});
        }
    }
    priorData.push({x:i, cycleTime:estimate});

    // build cycle time issue
    let uuid = generateUuid('issue');
    issues[uuid] = {
        id: uuid,
        requiresChanges: false,
        title: 'Robot Cycle Time',
        description: 'Robot Cycle Time',
        complete: false,
        focus: {id:program.id, type:'program'},
        graphData: {
            series: priorData,
            lineColors: ["#009e73"],
            xAxisLabel: 'Program Iteration',
            yAxisLabel: 'Cycle Time',
            title: '',
        }
    }

    return [issues, {cycleTime: estimate}];
}
// Use delay and machine wait primitives (delays can be tweaked if application acceptable)
export const findIdleTimeIssues = ({state, program, stats}) => {
    let issues = {};

    if (!program || program.properties.status !== STATUS.VALID) {
        return [issues, {}];
    }

    const estimate = idleTimeEstimate(state, program.properties.compiled["{}"].steps);

    // get prior values
    let priorData = [];
    let i = 0;
    for (i = 0; i < stats.length; i++) {
        if (stats[i].idleTime) {
            priorData.push({x:i, idleTime:stats[i].idleTime});
        }
    }
    priorData.push({x:i, idleTime:estimate});

    // build idle time issue
    let uuid = generateUuid('issue');
    issues[uuid] = {
        id: uuid,
        requiresChanges: false,
        title: 'Robot Idle Time',
        description: 'Robot Idle Time',
        complete: false,
        focus: {id:program.id, type:'program'},
        graphData: {
            series: priorData,
            lineColors: ["#009e73"],
            xAxisLabel: 'Program Iteration',
            yAxisLabel: 'Idle Time',
            title: '',
        }
    }

    return [issues, {idleTime: estimate}];
}
// Retrun on Investment
export const findReturnOnInvestmentIssues = ({state, program, stats, settings}) => {
    let issues = {};
    let newStats = {}

    if (!program || program.properties.status !== STATUS.VALID) {
        return [issues, {}];
    }

    // get prior values
    let priorData = [];
    let i = 0;
    for (i = 0; i < stats.length; i++) {
        if (stats[i].roi) {
            priorData.push({x:i, ROI:stats[i].roi});
        }
    }

    // some constant product value and cost
    const productValue = settings['productValue'].value;
    const productCost = settings['productCost'].value;

    // track the cost of wear and tear due to robot acceleration
    let wearTearCost = 0;

    // destructive acceleration level
    const errorAccelLevel = settings['roiAccelError'].value;

    const wearAndTear = (acceleration) => {
        return (Math.abs(acceleration) / errorAccelLevel) * 0.0001;
    }

    let trajectoryUpdate = false;
    let previousJoints = {};
    let previousVelocity = {};
    let previousTimeStep = 0;
    program.properties.compiled["{}"].steps.forEach(step => {
        // If the next action is a move trajectory, start evaluating the wear and tear on the moving joints
        if(step.type === STEP_TYPE.ACTION_START && "moveTrajectoryType" === state[step.source].type) {
            trajectoryUpdate = true;
        // Move trajectory ends, reset variables
        } else if(step.type === STEP_TYPE.ACTION_END && "moveTrajectoryType" === state[step.source].type) {
            previousJoints = {};
            trajectoryUpdate = false;
        }
        // Compute the wear and tear on the joints
        if(trajectoryUpdate && step.type === STEP_TYPE.SCENE_UPDATE) {
            // Setup the initial values for the movement
            if (Object.keys(previousJoints).length === 0) {
                previousTimeStep = step.time;
                Object.keys(step.data.joints).forEach(joint => {
                    previousJoints[joint] = step.data.joints[joint];
                    previousVelocity[joint] = 0;
                });
            } else {
                // Calculate wear and tear for the given movement
                Object.keys(step.data.joints).forEach(joint => {
                    let jointValue = step.data.joints[joint];
                    let timeDelta = (step.time - previousTimeStep) / 1000;
                    let jointVelocity = (jointValue - previousJoints[joint]) / timeDelta;
                    let jointAcceleration = (jointVelocity - previousVelocity[joint]) / timeDelta;
                    wearTearCost = wearTearCost + wearAndTear(jointAcceleration);

                    previousVelocity[joint] = jointVelocity;
                    previousJoints[joint] = jointValue;
                });

                previousTimeStep = step.time;
            }
        }
    });

    // roi = (net return / net cost) * 100%
    let roi = productValue / (productCost + wearTearCost) * 100;

    priorData.push({x:i, ROI:roi});

    // build roi issue
    if (wearTearCost > 0) {
        newStats = {roi: roi};
        let uuid = generateUuid('issue');
        issues[uuid] = {
            id: uuid,
            requiresChanges: false,
            title: 'Return on Investment',
            description: 'Return on Investment',
            complete: false,
            focus: {id:program.id, type:'program'},
            graphData: {
                series: priorData,
                lineColors: ["#009e73"],
                xAxisLabel: 'Program Iteration',
                yAxisLabel: 'ROI (%)',
                title: '',
            }
        }

    }

    return [issues, newStats];
}