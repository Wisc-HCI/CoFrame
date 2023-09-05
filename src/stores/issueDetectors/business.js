import frameStyles from "../../frameStyles";
import { ROOT_PATH, STEP_TYPE } from "../Constants";
import { generateUuid } from "../generateUuid"
import {idleTimeEstimate, durationEstimate} from "../helpers"


const cycleTimeDoc = `Robot cycle time is a measure of how long the entire program takes to complete. Because of concurrency, this is not simply the sum of all sequentially-organized actions, but rather the difference between the start time and the end of the final process, action, or event that occurs. 
To improve buisiness-related outcomes, consider the following:
- Faster robot motions will generally decrease cycle time, but at the possible expense of safety. Consider those tradeoffs when configuring motions.
- As much as possible, configure the execution of various [Processes](processType) to minimize the amount of time that the robot needs to wait for them to complete (see Idle Time).
The graph in the Review Panel shows how your program's cycle time has changed over your course of edits. Each time feedback is refreshed, the graph is updated with the most recent values.
`

const idleTimeDoc = `Robot idle time is a measure of how much the robot is inactive. This can come from explicit use of [delays](delayType), [waiting](processWaitType) for processes to finish, or other temporal logic constraints. Reducing idle time also generally reduces total Cycle Time. To reduce Idle Time, you can consider the following:
- Remove as many [delays](delayType) actions as possible/
- Instead of using [Process Wait](processWaitType) actions, consider filling that time with other robot activities that need to be done.
> [primary]The graph in the Review Panel shows how your program's idle time has changed over your course of edits. Each time feedback is refreshed, the graph is updated with the most recent values.
`

const roiDoc = `Return on Investment (ROI) is a measure of how much the robot is contributing to the efficacy of the process (e.g. decrease in Cycle Time), compared to the possible costs in maintenence. Specifically, it considers the cost of the parts being produced in a certain amount of time, compared to the wear and tear on the robot (approximated by joint movement of the robot over the course of a cycle). Programs that optimize the ROI will have the robot move as little as possible with the shortest cycle time.
To improve ROI, consider doing the following:
- Create efficient robot motions by considering when IK versus joint-based trajectories are most effective. 
- Cluster spatially-close activities together so the robot has fewer large movements to perform.
- Reduce Cycle Time annd Idle Time.
> [primary]The graph in the Review Panel shows how your program's ROI has changed over your course of edits. Each time feedback is refreshed, the graph is updated with the most recent values.
`

// Requires trace timing + delay and machine wait primitives (start computation after move_unplanned in initialize)
export const findCycleTimeIssues = ({program, stats, compiledData}) => {
    let issues = {};

    const estimate = durationEstimate(compiledData[program.id]?.[ROOT_PATH]?.steps);

    // get prior values
    let priorData = [];
    let i = 0;
    let peak = 0;
    for (i = 0; i < stats.length; i++) {
        if (stats[i].cycleTime !== null) {
            if (stats[i].cycleTime > peak) {
                peak = stats[i].cycleTime;
            }

            priorData.push({x:i, cycleTime:stats[i].cycleTime});
        }
    }
    if (estimate > peak) {
        peak = estimate;
    }
    priorData.push({x:i, cycleTime:estimate});

    // build cycle time issue
    let uuid = generateUuid('issue');
    issues[uuid] = {
        id: uuid,
        requiresChanges: false,
        title: 'Robot Cycle Time',
        description: 'Robot Cycle Time',
        featuredDocs: {[program.id]:cycleTimeDoc},
        complete: false,
        focus: [program.id],
        graphData: {
            series: priorData,
            xAxisLabel: 'Program Iteration',
            yAxisLabel: 'Cycle Time',
            thresholds: [
                {range: ["MIN","MAX"], color: frameStyles.colors["business"], label: 'OK'}
            ],
            units: 's',
            decimals: 5,
            title: '',
            isTimeseries: false
        }
    }

    return [issues, {cycleTime: estimate}];
}
// Use delay and machine wait primitives (delays can be tweaked if application acceptable)
export const findIdleTimeIssues = ({programData, program, stats, compiledData}) => {
    let issues = {};

    const estimate = idleTimeEstimate(programData, compiledData[program.id]?.[ROOT_PATH]?.steps);

    // get prior values
    let priorData = [];
    let i = 0;
    let peak = 0;
    for (i = 0; i < stats.length; i++) {
        if (stats[i].idleTime !== null) {
            if (stats[i].idleTime > peak) {
                peak = stats[i].idleTime;
            }
            priorData.push({x:i, idleTime:stats[i].idleTime});
        }
    }

    if (estimate > peak) {
        peak = estimate;
    }
    priorData.push({x:i, idleTime:estimate});

    // build idle time issue
    let uuid = generateUuid('issue');
    issues[uuid] = {
        id: uuid,
        requiresChanges: false,
        title: 'Robot Idle Time',
        description: 'Robot Idle Time',
        featuredDocs: {[program.id]:idleTimeDoc},
        complete: false,
        focus: [program.id],
        graphData: {
            series: priorData,
            xAxisLabel: 'Program Iteration',
            yAxisLabel: 'Idle Time',
            thresholds: [
                {range: ["MIN","MAX"], color: frameStyles.colors["business"], label: 'OK'}
            ],
            units: 's',
            decimals: 5,
            title: '',
            isTimeseries: false
        }
    }

    return [issues, {idleTime: estimate}];
}
// Retrun on Investment
export const findReturnOnInvestmentIssues = ({programData, program, stats, settings, compiledData}) => {
    let issues = {};
    let newStats = {}
    
    // get prior values
    let priorData = [];
    let i = 0;
    let peak = Number.MIN_VALUE;
    for (i = 0; i < stats.length; i++) {
        if (stats[i].roi !== null) {
            if (stats[i].roi > peak) {
                peak = stats[i].roi;
            }

            priorData.push({x:i, ROI:stats[i].roi});
        }
    }

    // some constant product value and cost
    const productValue = settings['productValue'].value;
    const productCost = settings['productCost'].value;
    const targetProduct = settings['product'].value;

    // track the cost of wear and tear due to robot acceleration
    let wearTearCost = 0;

    // destructive acceleration level
    const errorAccelLevel = settings['roiAccelError'].value;

    const wearAndTear = (acceleration) => {
        return (Math.abs(acceleration) / errorAccelLevel) * 0.0001;
    }

    let numberProductsCreated = 0;
    let trajectoryUpdate = false;
    let previousJoints = {};
    let previousVelocity = {};
    let previousTimeStep = 0;
    compiledData[program.id]?.[ROOT_PATH]?.steps?.forEach(step => {
        // Count the number of target products created (consumed)
        if (step.type === STEP_TYPE.SPAWN_ITEM && step.data.thing === targetProduct) {
            numberProductsCreated += 1;
        }

        // If the next action is a move trajectory, start evaluating the wear and tear on the moving joints
        if(step.type === STEP_TYPE.ACTION_START && "moveTrajectoryType" === programData[step.source].type) {
            trajectoryUpdate = true;
        // Move trajectory ends, reset variables
        } else if(step.type === STEP_TYPE.ACTION_END && "moveTrajectoryType" === programData[step.source].type) {
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
                    wearTearCost += wearAndTear(jointAcceleration);

                    previousVelocity[joint] = jointVelocity;
                    previousJoints[joint] = jointValue;
                });

                previousTimeStep = step.time;
            }
        }
    });

    // roi = (net return / net cost) * 100%
    let roi = (numberProductsCreated * productValue) / ((numberProductsCreated * productCost) + wearTearCost) * 100;

    if (roi > peak) {
        peak = roi;
    }
    priorData.push({x:i, ROI:roi});

    if (wearTearCost > 0) {
        // build roi issue
        newStats = {roi: roi};
        let uuid = generateUuid('issue');
        issues[uuid] = {
            id: uuid,
            requiresChanges: false,
            title: 'Return on Investment',
            description: 'Return on Investment',
            featuredDocs: {[program.id]:roiDoc},
            complete: false,
            focus: [program.id],
            graphData: {
                series: priorData,
                xAxisLabel: 'Program Iteration',
                yAxisLabel: 'ROI',
                thresholds: [
                    {range: ["MIN","MAX"], color: frameStyles.colors["business"], label: 'OK'}
                ],
                units: '%',
                decimals: 5,
                title: '',
                isTimeseries: false
            }

            
        }
    }

    return [issues, newStats];
}