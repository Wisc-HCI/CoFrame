import { generateUuid } from "../generateUuid"
import {idleTimeEstimate, durationEstimate} from "../helpers"

// Requires trace timing + delay and machine wait primitives (start computation after move_unplanned in initialize)
export const findCycleTimeIssues = ({program, unrolled, stats}) => {
    let issues = {};

    // TODO: Replace with Hunter's helper function
    //let randY = Math.random();
    const randY = durationEstimate(unrolled);
    

    // get prior values
    let priorData = [];
    let i = 0;
    for (i = 0; i < stats.length; i++) {
        if (stats[i].cycleTime) {
            priorData.push({x:i, cycleTime:stats[i].cycleTime});
        }
    }
    priorData.push({x:i, cycleTime:randY});

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
    
    return [issues, {cycleTime: randY}];
}
// Use delay and machine wait primitives (delays can be tweaked if application acceptable)
export const findIdleTimeIssues = ({program, unrolled, stats}) => {
    let issues = {};

    // TODO: Replace with Hunter's helper function
    const randY = idleTimeEstimate(unrolled);

    // get prior values
    let priorData = [];
    let i = 0;
    for (i = 0; i < stats.length; i++) {
        if (stats[i].idleTime) {
            priorData.push({x:i, idleTime:stats[i].idleTime});
        }
    }
    priorData.push({x:i, idleTime:randY});


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

    return [issues, {idleTime: randY}];
}
// Retrun on Investment
export const findReturnOnInvestmentIssues = ({program, unrolled, stats}) => { 
    let issues = {};

    // TODO: Replace with Hunter's helper function
    let randY = Math.random();

    // get prior values
    let priorData = [];
    let i = 0;
    for (i = 0; i < stats.length; i++) {
        if (stats[i].roi) {
            priorData.push({x:i, ROI:stats[i].roi});
        }
    }
    priorData.push({x:i, ROI:randY});

    // build roi issue
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
            yAxisLabel: 'ROI',
            title: '',
        }
    }

    return [issues, {roi: randY}];
}