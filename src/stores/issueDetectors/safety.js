import frameStyles from "../../frameStyles";
import { STATUS, STEP_TYPE } from "../Constants";
import { generateUuid } from "../generateUuid";
import { checkHandThresholds, stepsToEEPoseScores, likProximityAdjustment, getIDsAndStepsFromCompiled } from "../helpers";
import lodash from 'lodash';
import { DATA_TYPES } from "simple-vp";

const linkNames = ['shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link'];
const linkNameMap = {
    'shoulder_link': 'Shoulder Link',
    'upper_arm_link': 'Upper Arm Link',
    'forearm_link': 'Forearm Link',
    'wrist_1_link': 'Wrist 1 Link',
    'wrist_2_link': 'Wrist 2 Link',
    'wrist_3_link': 'Wrist 3 Link'
};
const linkColorMap = {
    'shoulder_link': '#009e9e',
    'upper_arm_link': '#9e0000',
    'forearm_link': '#9e0078',
    'wrist_1_link': '#9c9e00',
    'wrist_2_link': '#9e7100',
    'wrist_3_link': '#0b9e00'
};

const NO_ERROR_COLOR = {r: 255, g: 255, b: 255};
const WARNING_COLOR = {r: 204, g: 121, b: 167};
const ERROR_COLOR = {r: 233, g: 53, b: 152};

export const findEndEffectorPoseIssues = ({program, programData, settings}) => { // Requires trace pose information
    let issues = {};

    let warningLevel = settings['eePoseWarn'].value;
    let errorLevel = settings['eePoseErr'].value;

    // TODO: these should be arrays
    let gripper = lodash.filter(programData, function (v) { return v.type === 'gripperType'})[0];
    let robotAgent = lodash.filter(programData, function (v) { return v.type === 'robotAgentType'})[0];

    let res = getIDsAndStepsFromCompiled(program, programData, STEP_TYPE.SCENE_UPDATE, "moveTrajectoryType");
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
            toolFrames[step.source].push(step.data.links.tool0.position);
            // TODO: iterate over robots and tools
            endPointFrames[step.source].push(step.data.goalPoses[robotAgent.id][gripper.id].position);
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
                endEffectorScores.push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: ERROR_COLOR});
            } else if (scores[i] >= warningLevel) {
                shouldReportIssue = true;
                endEffectorScores.push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: WARNING_COLOR});
            } else {
                endEffectorScores.push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: NO_ERROR_COLOR});
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
                        {range: [warningLevel, errorLevel], color: frameStyles.colors["safety"], label: 'Warning'},
                        {range: [errorLevel, "MAX"], color: frameStyles.errorColors["safety"], label: 'Error'},
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
export const findCollisionIssues = ({program, programData, settings}) => { 
    let issues = {};
    const warningLevel = settings['collisionWarn'].value;
    const errorLevel = settings['collisionErr'].value;

    let res = getIDsAndStepsFromCompiled(program, programData, STEP_TYPE.SCENE_UPDATE, "moveTrajectoryType");
    let moveTrajectoryIDs = res[0];
    let sceneUpdates = res[1];

    let robotAgent = lodash.filter(programData, function (v) { return v.type === 'robotAgentType'})[0];
    let robotPoints = robotAgent ? robotAgent.properties.pinchPointPairLinks : [];
    let fixtureIDs = lodash.filter(programData, function (v) { return v.type === 'fixtureType'}).map(fixture => { return fixture.id });

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
    robotJointIDs.forEach(rID => {
        fixtureIDs.forEach(fID => {
            envrionTracker.push({link1: rID, link2: fID});
        })
    });

    let timeData = {};
    let sCol = {};
    let eCol = {};
    let positionData = {};
    sceneUpdates.forEach(step => {
        if (step.source && moveTrajectoryIDs.includes(step.source)) {
            if (!(step.source in timeData)) {
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
                if (shouldGraphJoint[selfIndex][rLink]) {
                    let curFrame = positionData[moveID][i][rLink].position;
                    shouldGraphSet[selfIndex] = true;

                    if (!collisionData[selfIndex][rLink]) {
                        collisionData[selfIndex][rLink] = [];
                    }

                    if (sCol[moveID][i][rLink] <= errorLevel) {
                        collisionErrors[selfIndex] = true;
                        collisionData[selfIndex][rLink].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: ERROR_COLOR});
                    } else if (sCol[moveID][i][rLink] <= warningLevel) {
                        collisionData[selfIndex][rLink].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: WARNING_COLOR});
                    } else {
                        collisionData[selfIndex][rLink].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: NO_ERROR_COLOR});
                    }
                    graphData[selfIndex][i][linkNameMap[rLink]] = sCol[moveID][i][rLink];
                }
            });

            Object.keys(eCol[moveID][i]).forEach(rLink => {
                if (shouldGraphJoint[envIndex][rLink]) {
                    let curFrame = positionData[moveID][i][rLink].position;
                    shouldGraphSet[envIndex] = true;

                    if (!collisionData[envIndex][rLink]) {
                        collisionData[envIndex][rLink] = [];
                    }

                    if (eCol[moveID][i][rLink] <= errorLevel) {
                        collisionErrors[envIndex] = true;
                        collisionData[envIndex][rLink].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: ERROR_COLOR});
                    } else if (eCol[moveID][i][rLink] <= warningLevel) {
                        collisionData[envIndex][rLink].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: WARNING_COLOR});
                    } else {
                        collisionData[envIndex][rLink].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: NO_ERROR_COLOR});
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
                        {range: [warningLevel, errorLevel], color: frameStyles.colors["safety"], label: 'Warning'},
                        {range: [errorLevel, "MAX"], color: frameStyles.errorColors["safety"], label: 'Error'},
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
                        {range: [warningLevel, errorLevel], color: frameStyles.colors["safety"], label: 'Warning'},
                        {range: [errorLevel, "MAX"], color: frameStyles.errorColors["safety"], label: 'Error'},
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
export const findOccupancyIssues = ({program, programData, settings}) => {
    let issues = {};

    const warningLevel = settings['occupancyWarn'].value;
    const errorLevel = settings['occupancyErr'].value;

    let res = getIDsAndStepsFromCompiled(program, programData, STEP_TYPE.SCENE_UPDATE, "moveTrajectoryType");
    let moveTrajectoryIDs = res[0];
    let sceneUpdates = res[1];

    let timeData = {};
    let proximityData = {};
    let positionData = {};
    let shouldGraphlink = {};
    let occupancyValues = {};

    sceneUpdates.forEach(step => {
        if (step.source && moveTrajectoryIDs.includes(step.source)) {
            if (!(step.source in timeData)) {
                timeData[step.source] = [];
                proximityData[step.source] = [];
                positionData[step.source] = []
                
                occupancyValues[step.source] = {};
                shouldGraphlink[step.source] = [];
                for (let i = 0; i < linkNames.length; i++) {
                    shouldGraphlink[step.source].push(false);
                    occupancyValues[step.source][linkNames[i]] = [];
                }
            }
            positionData[step.source].push({...step.data.links});
            timeData[step.source].push(step.time);
            proximityData[step.source].push(likProximityAdjustment([], step.data.proximity.filter(v => !v.physical), false))
        }
    });

    moveTrajectoryIDs.forEach(moveID => {
        // Figure out which links to graph
        for (let i = 0; i < proximityData[moveID].length; i++) {
            for (let j = 0; j < linkNames.length; j++) {
                if (!shouldGraphlink[moveID][j] && proximityData[moveID][i][linkNames[j]]) {
                    Object.values(proximityData[moveID][i][linkNames[j]]).forEach(obj => {
                        if (obj.distance <= warningLevel) {
                                shouldGraphlink[moveID][j] = true;
                                j = linkNames.length;
                        }
                    });
                }
            }
        }
        let enteredZone = false;
        let filteredGraphData = [];

        for (let j = 0; j < proximityData[moveID].length; j++) {
            let dataPoint = {x: timeData[moveID][j]};
            for (let i = 0; i < shouldGraphlink[moveID].length; i++) {
                // Grab the position data for each link and mark it's color
                let curFrame = positionData[moveID][j][linkNames[i]].position;
                let point = null;

                if (proximityData[moveID][j][linkNames[i]]) {
                    Object.values(proximityData[moveID][j][linkNames[i]]).forEach(obj => {
                        if (obj.distance <= errorLevel) {
                            enteredZone = true;
                            occupancyValues[moveID][linkNames[i]].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: ERROR_COLOR});
                        } else if (obj.distance <= warningLevel) {
                            occupancyValues[moveID][linkNames[i]].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: WARNING_COLOR});
                        } else {
                            occupancyValues[moveID][linkNames[i]].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: NO_ERROR_COLOR});
                        }
                        point = obj.distance;
                    });
                }

                // Add the data for the contextual info graph
                if (shouldGraphlink[moveID][i] && point !== null) {
                    dataPoint[linkNameMap[linkNames[i]]] = point;
                }
            }

            if (Object.keys(dataPoint).length > 1) {
                filteredGraphData.push(dataPoint);
            }
        }

        let linkColors = [];
        for (let i = 0; i < linkNames.length; i++) {
            if (shouldGraphlink[moveID][i]) {
                linkColors.push(linkColorMap[linkNames[i]]);
            }
        }

        if (filteredGraphData.length > 0) {
            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
                requiresChanges: false,
                title: enteredZone ? `Entered Occupancy Zone`: `Close to Occupancy Zone`,
                description: enteredZone ? `Robot trajectory entered occupancy zone`: `Robot trajectory results in close proximity to the occupancy zone`,
                complete: false,
                focus: [moveID],
                graphData: {
                    series: filteredGraphData,
                    xAxisLabel: 'Timestamp',
                    yAxisLabel: 'Proximity',
                    title: '',
                    thresholds: [
                        {range: ["MIN", errorLevel], color: frameStyles.errorColors["safety"], label: 'Error'},
                        {range: [errorLevel, warningLevel], color: frameStyles.colors["safety"], label: 'Warning'},
                        {range: [warningLevel, "MAX"], color: 'grey', label: 'OK'},
                    ],
                    units: 'm',
                    decimal: 5,
                    isTimeseries: true
                },
                sceneData: {vertices: occupancyValues[moveID]}
            }
        }
    });

    return [issues, {}];
}

export const findPinchPointIssues = ({program, programData}) => { // Requires pinch-point graders
    let issues = {};
    let addressed = [];

    let res = getIDsAndStepsFromCompiled(program, programData, STEP_TYPE.SCENE_UPDATE, "moveTrajectoryType");
    let moveTrajectoryIDs = res[0];
    let sceneUpdates = res[1];

    sceneUpdates.forEach((step) => {
        if (step.source && moveTrajectoryIDs.includes(step.source) && !addressed.includes(step.source)) {
            let hasError = false;
            let proxData = step.data?.proximity;
            if (proxData) {
                proxData.forEach(entry => {
                    hasError = hasError || (entry.distance !== null && checkHandThresholds(entry.distance));
                });
            }

            if (hasError) {
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    id: uuid,
                    requiresChanges: true,
                    title: `Likely pinch points`,
                    description: `Robot trajectory includes likely pinch points`,
                    complete: false,
                    focus: [step.source],
                    graphData: null,
                    sceneData: null
                }

                addressed.push(step.source);
            }
        }
    });

    return [issues, {}];
}

export const findThingMovementIssues = ({programData}) => { // May require trace pose information
    let issues = {};

    Object.values(programData).filter(v => v.type === 'moveGripperType').forEach(primitive=>{
        if (primitive.properties.thing && primitive.properties.positionEnd < primitive.properties.positionStart) {
            const thing = programData[programData[primitive.properties.thing].ref];
            if (!thing.properties.safe) {
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    id: uuid,
                    requiresChanges: true,
                    title: `Grasping on unsafe object`, // might need some changes
                    description: `The robot is grasping on a dangerous item`,// might need some changes
                    complete: false,
                    focus: [primitive.id],
                    graphData: null
                }
            }
        }
    });


    return [issues, {}];
}