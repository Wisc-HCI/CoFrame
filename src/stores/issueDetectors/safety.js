import frameStyles from "../../frameStyles";
import {  ROOT_PATH, STEP_TYPE } from "../Constants";
import { generateUuid } from "../generateUuid";
import { checkHandThresholds, stepsToEEPoseScores, getIDsAndStepsFromCompiled } from "../helpers";
import { likProximityAdjustment } from "../../helpers/conversion";
import lodash from 'lodash';
import { hexToRgb } from "../../helpers/colors";
import { queryWorldPose, updateEnvironModel } from "../../helpers/geometry";

export const findEndEffectorPoseIssues = ({program, programData, settings, compiledData}) => { // Requires trace pose information
    let issues = {};

    let warningLevel = settings['eePoseWarn'].value;
    let errorLevel = settings['eePoseErr'].value;

    // TODO: these should be arrays
    let gripper = lodash.filter(programData, function (v) { return v.type === 'gripperType'})[0];
    let robotAgent = lodash.filter(programData, function (v) { return v.type === 'robotAgentType'})[0];

    let res = getIDsAndStepsFromCompiled(program, programData, STEP_TYPE.SCENE_UPDATE, "moveTrajectoryType", compiledData);
    let moveTrajectoryIDs = res[0];
    let sceneUpdates = res[1];


    let toolFrames = {};
    let endPointFrames = {};
    let timeData = {};
    sceneUpdates.forEach(step => {
        if (step.source && moveTrajectoryIDs.includes(step.source)) {
            if (!(step.source in timeData)) {
                timeData[step.source] = [];
                toolFrames[step.source] = [];
                endPointFrames[step.source] = [];
            }
            timeData[step.source].push(step.time);
            toolFrames[step.source].push(step.data.links[gripper.properties.relativeTo].position);
            // TODO: iterate over robots and tools
            endPointFrames[step.source].push(step.data.attachmentPoses[robotAgent.id][gripper.id].position);
        }
    });

    moveTrajectoryIDs.forEach(moveID => {
        // TODO: score each robot/tool combination
        let scores = stepsToEEPoseScores(toolFrames[moveID], endPointFrames[moveID]);
        let endEffectorScores = [];
        let graphData = [];
        let hasError = false;
        let shouldReportIssue = false;

        for (let i = 0; i < toolFrames[moveID].length; i++) {
            let curFrame = toolFrames[moveID][i]
            if (scores[i] >= errorLevel) {
                hasError = true;
                shouldReportIssue = true;
                endEffectorScores.push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: hexToRgb(frameStyles.errorColors["safety"])});
            } else if (scores[i] >= warningLevel) {
                shouldReportIssue = true;
                endEffectorScores.push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: hexToRgb(frameStyles.colors["safety"])});
            } else {
                endEffectorScores.push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: hexToRgb(frameStyles.colors["default"])});
            }
            graphData.push({x: timeData[moveID][i] / 1000, endEffectorScore: scores[i]});
        }

        if (shouldReportIssue) {
            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
                requiresChanges: hasError,
                title: `End effector pose is poor`,
                description: `End effector pose is poor`,
                complete: false,
                focus: [moveID],
                graphData: {
                    series: graphData,
                    xAxisLabel: 'Timestamp',
                    yAxisLabel: 'Pose Score',
                    title: '',
                    thresholds: [
                        {range: ["MIN", warningLevel], color: 'grey', label: 'OK'},
                        {range: [warningLevel, errorLevel], color: hexToRgb(frameStyles.colors["safety"]), label: 'Warning'},
                        {range: [errorLevel, "MAX"], color: hexToRgb(frameStyles.errorColors["safety"]), label: 'Error'},
                    ],
                    units: '',
                    decimal: 5,
                    isTimeseries: true
                },
                sceneData: {vertices: {endEffectorPose: endEffectorScores}}
            }
        }
    });

    return [issues, {}];
}

// Requires collision graders
export const findCollisionIssues = ({program, programData, settings, environmentModel, compiledData}) => { 
    let issues = {};
    return [issues, {}];

    const warningLevel = settings['collisionWarn'].value;
    const errorLevel = settings['collisionErr'].value;

    const robotAgent = lodash.filter(programData, function (v) { return v.type === 'robotAgentType'})[0];
    const robotPoints = robotAgent ? robotAgent.properties.pinchPointPairLinks : [];

    const linkNames = Object.values(robotAgent.properties.jointLinkMap);
    let linkNameMap = {}
    linkNames.forEach(name => {
        linkNameMap[name] = name.replace(/\w\S*/g, (w) => (w.replace(/^\w/, (c) => c.toUpperCase())));
    });

    // Build pairing for environment tracking
    let robotJointIDs = [];
    robotPoints.forEach(pair => {
        if (!(robotJointIDs.includes[pair.link1])) {
            robotJointIDs.push(pair.link1);
        }
        if (!(robotJointIDs.includes[pair.link2])) {
            robotJointIDs.push(pair.link2);
        }
    });

    let envrionTracker = [];
    const collisionIDs = lodash.filter(programData, function (v) { return v.type === 'collisionShapeType' }).map(collision => { return collision.id });
    robotJointIDs.forEach(rID => {
        collisionIDs.forEach(fID => {
            envrionTracker.push({link1: rID, link2: fID});
        })
    });

    let timeData = {};
    let sCol = {};
    let eCol = {};
    let positionData = {};
    let moveTrajectoryIDs = [];

    compiledData[program.id]?.[ROOT_PATH]?.steps?.forEach(step => {
        if (step.type === STEP_TYPE.SCENE_UPDATE && programData[step.source]?.type === "moveTrajectoryType") {
            if (!(step.source in timeData)) {
                moveTrajectoryIDs.push(step.souce);
                timeData[step.source] = [];
                sCol[step.source] = [];
                eCol[step.source] = [];
                positionData[step.source] = [];
            }
            timeData[step.source].push(step.time);
            positionData[step.source].push({...step.data.links});

            let sColTmp = likProximityAdjustment(robotPoints, step.data.proximity, true);
            let sColAdjusted = {}
            // For each joint, assign it the minimum proximity value
            Object.keys(sColTmp).forEach(key => {
                if (linkNames.includes(key)) {
                    sColAdjusted[key] = Math.min(...Object.values(sColTmp[key]).map(v => {return v.distance}));
                }
            })
            sCol[step.source].push(sColAdjusted);

            // Environment Collisions
            let eColTmp = likProximityAdjustment(envrionTracker, step.data.proximity, true);
            let eColAdjusted = {}
            // For each joint, assign it the minimum proximity value
            Object.keys(eColTmp).forEach(key => {
                if (linkNames.includes(key)) {
                    eColAdjusted[key] = Math.min(...Object.values(eColTmp[key]).map(v => {return v.distance}));
                }
            })
            eCol[step.source].push(eColAdjusted);
        }
    });

    moveTrajectoryIDs.forEach(moveID => {
        const selfIndex = 0;
        const envIndex = 1;
        let collisionErrors = [false, false];

        // Iteratively build the graph and scene data
        let graphData = [[], []];
        let collisionData = [{}, {}];
        let shouldGraphSet = [false, false];
        let shouldGraphJoint = [{}, {}];

        // For each joint (at each timestep), determine if it's within the warning threshold
        for (let i = 0; i < timeData[moveID].length; i++) {
            Object.keys(sCol[moveID][i]).forEach(rLink => {
                if (!(rLink in shouldGraphJoint[selfIndex])) {
                    shouldGraphJoint[selfIndex][rLink] = false;
                }

                if (sCol[moveID][i][rLink] <= warningLevel) {
                    shouldGraphJoint[selfIndex][rLink] = true;
                }
            });

            Object.keys(eCol[moveID][i]).forEach(rLink => {
                if (!(rLink in shouldGraphJoint[envIndex])) {
                    shouldGraphJoint[envIndex][rLink] = {};
                }

                if (eCol[moveID][i][rLink] <= warningLevel) {
                    shouldGraphJoint[envIndex][rLink] = true;
                }
            });
        }

        // Build the graph and collision data
        // If a joint should be graphed, add it's data points to the respective array
        for (let i = 0; i < timeData[moveID].length; i++) {
            let timestamp = timeData[moveID][i] / 1000;
            graphData[selfIndex].push({x: timestamp});
            graphData[envIndex].push({x: timestamp});

            Object.keys(sCol[moveID][i]).forEach(rLink => { 
                environmentModel = updateEnvironModel(environmentModel, rLink, positionData[moveID][i][rLink].position, positionData[moveID][i][rLink].rotation)
            });

            let posRot = {};
            Object.keys(sCol[moveID][i]).forEach(rLink => { 
                posRot[rLink] = queryWorldPose(environmentModel, rLink, '');
            });

            Object.keys(sCol[moveID][i]).forEach(rLink => {
                if (shouldGraphJoint[selfIndex][rLink]) {
                    let curFrame = posRot[rLink].position;
                    shouldGraphSet[selfIndex] = true;

                    if (!collisionData[selfIndex][rLink]) {
                        collisionData[selfIndex][rLink] = [];
                    }

                    if (sCol[moveID][i][rLink] <= errorLevel) {
                        collisionErrors[selfIndex] = true;
                        collisionData[selfIndex][rLink].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: hexToRgb(frameStyles.errorColors["safety"])});
                    } else if (sCol[moveID][i][rLink] <= warningLevel) {
                        collisionData[selfIndex][rLink].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: hexToRgb(frameStyles.colors["safety"])});
                    } else {
                        collisionData[selfIndex][rLink].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: hexToRgb(frameStyles.colors["default"])});
                    }
                    graphData[selfIndex][i][linkNameMap[rLink]] = sCol[moveID][i][rLink];
                }
            });

            Object.keys(eCol[moveID][i]).forEach(rLink => {
                if (shouldGraphJoint[envIndex][rLink]) {
                    let curFrame = posRot[rLink].position;
                    shouldGraphSet[envIndex] = true;

                    if (!collisionData[envIndex][rLink]) {
                        collisionData[envIndex][rLink] = [];
                    }

                    if (eCol[moveID][i][rLink] <= errorLevel) {
                        collisionErrors[envIndex] = true;
                        collisionData[envIndex][rLink].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: hexToRgb(frameStyles.errorColors["safety"])});
                    } else if (eCol[moveID][i][rLink] <= warningLevel) {
                        collisionData[envIndex][rLink].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: hexToRgb(frameStyles.colors["safety"])});
                    } else {
                        collisionData[envIndex][rLink].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: hexToRgb(frameStyles.colors["default"])});
                    }
                    graphData[envIndex][i][linkNameMap[rLink]] = eCol[moveID][i][rLink];
                }
            });
        }

         // Build issue for self collisions
         if (shouldGraphSet[selfIndex]) {
            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
                requiresChanges: collisionErrors[selfIndex],
                title: collisionErrors[selfIndex] ? `Robot collides with self` : `Robot is in near collision with self`,
                description: `Robot collides with self`,
                complete: false,
                focus: [moveID],
                graphData: {
                    series: graphData[selfIndex],
                    xAxisLabel: 'Timestamp',
                    yAxisLabel: 'Proximity',
                    title: '',
                    thresholds: [
                        {range: ["MIN", warningLevel], color: 'grey', label: 'OK'},
                        {range: [warningLevel, errorLevel], color: hexToRgb(frameStyles.colors["safety"]), label: 'Warning'},
                        {range: [errorLevel, "MAX"], color: hexToRgb(frameStyles.errorColors["safety"]), label: 'Error'},
                    ],
                    units: 'm',
                    decimal: 5,
                    isTimeseries: true
                },
                sceneData: {vertices: collisionData[selfIndex]}
            }
        }

        // Build issue for environmental collisions
        if (shouldGraphSet[envIndex]) {
            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
                requiresChanges: collisionErrors[selfIndex],
                title: collisionErrors[selfIndex] ? `Robot collides with environment` : `Robot is in near collision with environment`,
                description: `Robot collides with the environment`,
                complete: false,
                focus: [moveID],
                graphData: {
                    series: graphData[envIndex],
                    xAxisLabel: 'Timestamp',
                    yAxisLabel: 'Proximity',
                    title: '',
                    thresholds: [
                        {range: ["MIN", warningLevel], color: 'grey', label: 'OK'},
                        {range: [warningLevel, errorLevel], color: hexToRgb(frameStyles.colors["safety"]), label: 'Warning'},
                        {range: [errorLevel, "MAX"], color: hexToRgb(frameStyles.errorColors["safety"]), label: 'Error'},
                    ],
                    units: 'm',
                    decimal: 5,
                    isTimeseries: true
                },
                sceneData: {vertices: collisionData[envIndex]}
            }
        }
    });

    return [issues, {}];
}

// Requires occupancy zone graders
export const findOccupancyIssues = ({program, programData, settings, environmentModel, compiledData}) => {
    let issues = {};

    const warningLevel = settings['occupancyWarn'].value;
    const errorLevel = settings['occupancyErr'].value;

    let timeData = {};
    let proximityData = {};
    let positionData = {};
    let shouldGraphlink = {};
    let occupancyValues = {};

    let robotAgent = lodash.filter(programData, function (v) { return v.type === 'robotAgentType' })[0];
    let linkNames = Object.values(robotAgent.properties.jointLinkMap);
    let linkNameMap = {}
    linkNames.forEach(name => {
        linkNameMap[name] = name.replace(/\w\S*/g, (w) => (w.replace(/^\w/, (c) => c.toUpperCase())));
    });

    // Build timeline of move trajectory steps
    let moveIds = [];
    compiledData[program.id]?.[ROOT_PATH]?.steps?.forEach(step => {
        if (step.type === STEP_TYPE.SCENE_UPDATE && programData[step.source]?.type === 'moveTrajectoryType') {
            if (!(moveIds.includes(step.source))) {
                moveIds.push(step.source);
                
                timeData[step.source] = [];
                proximityData[step.source] = [];
                positionData[step.source] = [];
                
                occupancyValues[step.source] = {};
                shouldGraphlink[step.source] = [];
                for (let i = 0; i < linkNames.length; i++) {
                    shouldGraphlink[step.source].push(false);
                    occupancyValues[step.source][linkNames[i]] = [];
                }
            }
            positionData[step.source].push({...step.data.links});
            timeData[step.source].push(step.time);
            proximityData[step.source].push(likProximityAdjustment([], step.data.proximity.filter(v => !v.physical), false));
        }
    });

    moveIds.forEach(source => {
        // Figure out which links to graph
        for (let i = 0; i < proximityData[source].length; i++) {
            for (let j = 0; j < linkNames.length; j++) {
                if (!shouldGraphlink[source][j] && proximityData[source][i][linkNames[j]]) {
                    Object.values(proximityData[source][i][linkNames[j]]).forEach(obj => {
                        if (obj.distance <= warningLevel) {
                                shouldGraphlink[source][j] = true;
                                j = linkNames.length;
                        }
                    });
                }
            }
        }
        let enteredZone = false;
        let filteredGraphData = [];


        for (let j = 0; j < proximityData[source].length; j++) {
            let dataPoint = {x: timeData[source][j]};
            for (let i = 0; i < shouldGraphlink[source].length; i++) {
                // Grab the position data for each link and mark it's color
                environmentModel = updateEnvironModel(environmentModel, linkNames[i], positionData[source][j][linkNames[i]].position, positionData[source][j][linkNames[i]].rotation)
                const posRot = queryWorldPose(environmentModel, linkNames[i], '');
                let curFrame = posRot.position;
                let point = null;

                if (proximityData[source][j][linkNames[i]]) {
                    Object.values(proximityData[source][j][linkNames[i]]).forEach(obj => {
                        if (obj.distance <= errorLevel) {
                            enteredZone = true;
                            occupancyValues[source][linkNames[i]].push({position: {...curFrame}, color: hexToRgb(frameStyles.errorColors["safety"])});
                        } else if (obj.distance <= warningLevel) {
                            occupancyValues[source][linkNames[i]].push({position: {...curFrame}, color: hexToRgb(frameStyles.colors["safety"])});
                        } else {
                            occupancyValues[source][linkNames[i]].push({position: {...curFrame}, color: hexToRgb(frameStyles.colors["default"])});
                        }
                        point = obj.distance;
                    });
                }

                // Add the data for the contextual info graph
                if (shouldGraphlink[source][i] && point !== null) {
                    dataPoint[linkNameMap[linkNames[i]]] = point;
                }
            }

            if (Object.keys(dataPoint).length > 1) {
                filteredGraphData.push(dataPoint);
            }
        }

        if (filteredGraphData.length > 0) {
            const id = generateUuid('issue');
            issues[id] = {
                id: id,
                requiresChanges: false,
                title: enteredZone ? `Entered Occupancy Zone`: `Close to Occupancy Zone`,
                description: enteredZone ? `Robot trajectory entered occupancy zone`: `Robot trajectory results in close proximity to the occupancy zone`,
                complete: false,
                focus: [source],
                graphData: {
                    series: filteredGraphData,
                    xAxisLabel: 'Timestamp',
                    yAxisLabel: 'Proximity',
                    title: '',
                    thresholds: [
                        {range: ["MIN", errorLevel], color: hexToRgb(frameStyles.errorColors["safety"]), label: 'Error'},
                        {range: [errorLevel, warningLevel], color: hexToRgb(frameStyles.colors["safety"]), label: 'Warning'},
                        {range: [warningLevel, "MAX"], color: 'grey', label: 'OK'},
                    ],
                    units: 'm',
                    decimal: 5,
                    isTimeseries: true
                },
                sceneData: {vertices: occupancyValues[source]}
            }
        }
    });

    return [issues, {}];
}

export const findPinchPointIssues = ({program, programData, compiledData}) => { // Requires pinch-point graders
    let issues = {};
    let addressed = [];

    let pinchPoints = {};
    let timeData = {};
    let errorSteps = [];

    compiledData[program.id]?.[ROOT_PATH]?.steps?.forEach(step => {
        if (step.type === STEP_TYPE.SCENE_UPDATE && programData[step.source].type === 'moveTrajectoryType') {
            let hasError = false;
            let proxData = step.data?.proximity;
            if (proxData) {
                proxData.forEach(entry => {
                    hasError = hasError || (entry.distance !== null && checkHandThresholds(entry.distance));
                });
            }

            if (!pinchPoints[step.source]) {
                timeData[step.source] = [step.time];
                pinchPoints[step.source] = [hasError];
            } else {
                timeData[step.source].push(step.time);
                pinchPoints[step.source].push(hasError);
            }

            if (hasError) {
                errorSteps.push(step.source)
            }
        }
    });

    errorSteps.forEach(source => {
        if (!addressed.includes(source)) {
            const graphData = timeData[source].map((element, idx, arr) => {
                return {x: element, inPinch: (pinchPoints[source][idx] ? 1 : 0)}
            });

            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
                requiresChanges: true,
                title: `Likely pinch points`,
                description: `Robot trajectory includes likely pinch points`,
                complete: false,
                focus: [source],
                graphData: {
                    series: graphData,
                    xAxisLabel: 'Timestamp',
                    yAxisLabel: 'Pinch Points Exist',
                    units: '',
                    thresholds: [
                        {range: ["MIN", 1], color: 'grey', label: 'OK'},
                        {range: [1, "MAX"], color: hexToRgb(frameStyles.errorColors["safety"]), label: 'Error'}
                    ],
                    decimal: 1,
                    title: '',
                    isTimeseries: true
                },
                sceneData: null
            }

            addressed.push(source);
        }
    })

    return [issues, {}];
}

export const findThingSafetyIssues = ({program, programData, compiledData}) => { // May require trace pose information
    let issues = {};
    let trackedIds = [];

    compiledData[program.id]?.[ROOT_PATH]?.steps?.forEach(step => {
        let source = programData[step.source];

        if (!trackedIds.includes(step.source) &&
            step.type === STEP_TYPE.SCENE_UPDATE &&
            source.type === 'moveGripperType' &&
            source.properties.thing &&
            source.properties.positionEnd < source.properties.positionStart) {
                trackedIds.push(step.source);
                let thing = programData[step.data.thing.id];
                if (!thing.properties.safe) {
                    const uuid = generateUuid('issue');
                    issues[uuid] = {
                        id: uuid,
                        requiresChanges: true,
                        title: `Grasping an unsafe object`, // might need some changes
                        description: `The robot is grasping a dangerous item`,// might need some changes
                        complete: false,
                        focus: [source.id],
                        graphData: null
                    }
                }
        }
    });

    return [issues, {}];
}
