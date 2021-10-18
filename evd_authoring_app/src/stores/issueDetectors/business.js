import { generateUuid } from "../generateUuid"
import {idleTimeEstimate, durationEstimate} from "../helpers"

// Requires trace timing + delay and machine wait primitives (start computation after move_unplanned in initialize)
export const findCycleTimeIssues = ({program, unrolled, stats}) => {
    let issues = {};

    if (!unrolled) {
        return [issues, {}];
    }

    const estimate = durationEstimate(unrolled);

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
        uuid: uuid,
        requiresChanges: false,
        title: 'Robot Cycle Time',
        description: 'Robot Cycle Time',
        complete: false,
        focus: {uuid:program.uuid, type:'program'},
        graphData: {
            series: priorData,
            lineColors: ["#009e73"],
            xAxisLabel: 'Program Iteration',
            yAxisLabel: 'Cycle Time',
            title: '',
        }
    }
    console.log('cycle time')
    return [issues, {cycleTime: estimate}];
}
// Use delay and machine wait primitives (delays can be tweaked if application acceptable)
export const findIdleTimeIssues = ({program, unrolled, stats}) => {
    let issues = {};

    if (!unrolled) {
        return [issues, {}];
    }

    const estimate = idleTimeEstimate(unrolled);

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
        uuid: uuid,
        requiresChanges: false,
        title: 'Robot Idle Time',
        description: 'Robot Idle Time',
        complete: false,
        focus: {uuid:program.uuid, type:'program'},
        graphData: {
            series: priorData,
            lineColors: ["#009e73"],
            xAxisLabel: 'Program Iteration',
            yAxisLabel: 'Idle Time',
            title: '',
        }
    }
    console.log('idle time')
    return [issues, {idleTime: estimate}];
}
// Retrun on Investment
export const findReturnOnInvestmentIssues = ({program, unrolled, stats, settings}) => { 
    let issues = {};
    let newStats = {}

    if (!unrolled) {
        return [issues, {}];
    }

    const jointNames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'];

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

    Object.values(unrolled).forEach(primitive=>{
        if (primitive.type === 'node.primitive.move-trajectory.' ){
            
            let trajectory = primitive.parameters.trajectory_uuid;
            let jointData = trajectory.trace.joint_data;
            let timeData = trajectory.trace.time_data;

            let jointDataLength = jointData[jointNames[0]].length;

            // Calculate the acceleration and determine the cost from the acceleration
            for (let i = 0; i < jointNames.length; i++) {
                let prevVel = 0;
                for (let j = 1; j < jointDataLength; j++) {
                    let nextVel = (jointData[jointNames[i]][j] - jointData[jointNames[i]][j-1]) / (timeData[j] - timeData[j-1]);
                    let accel = (nextVel - prevVel) / (timeData[j] - timeData[j-1]);
                    wearTearCost = wearTearCost + wearAndTear(accel);
                    prevVel = nextVel;
                }
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
            uuid: uuid,
            requiresChanges: false,
            title: 'Return on Investment',
            description: 'Return on Investment',
            complete: false,
            focus: {uuid:program.uuid, type:'program'},
            graphData: {
                series: priorData,
                lineColors: ["#009e73"],
                xAxisLabel: 'Program Iteration',
                yAxisLabel: 'ROI (%)',
                title: '',
            }
        }

    }
    console.log('roi')
    return [issues, newStats];
}