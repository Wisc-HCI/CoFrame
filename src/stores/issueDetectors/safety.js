import frameStyles from "../../frameStyles";
import {  ROOT_PATH, STEP_TYPE } from "../Constants";
import { generateUuid } from "../generateUuid";
import { checkHandThresholds, stepsToEEPoseScores, getIDsAndStepsFromCompiled } from "../helpers";
import { likProximityAdjustment } from "../../helpers/conversion";
import lodash from 'lodash';
import { hexToRgb } from "../../helpers/colors";
import { queryWorldPose, updateEnvironModel, updateEnvironModelQuaternion } from "../../helpers/geometry";

const endEffectorPoseDoc = `During this action, it appears that the gripper may be moving in a potentially dangerous manner. Specifically, this involves cases where the gripper moves quickly in the direction of its outstretched fingers, such that they could result in problems if coming in contact with a worker's soft tissue. To address this issue, consider one or more of the following changes:
- Slow down the action so the movement of the gripper is not as fast, and therefore less of a danger
- Reorient the gripper during the action so that the broad side of the gripper is moving in the current direction for the majority of the action.
`;

const collisionNearSelfDoc = `During this action, it appears that the robot is close to colliding with itself. While this doesn't require changes, it is good practice to avoid these scenarios, as future issues may arise. To observe where this near collision is happening, observe the timeline, which should highlight sections of the trajectory where the issue occurs. To address this issue, consider one or more of the following changes:
- Adding waypoints to the trajectory to try and move away from the near collision point
- Moving existing waypoints and locations to move the robot in a way that avoids the near collision point.
- Switching trajectory types (IK or Joint)
`;

const collisionSelfDoc = `During this action, it appears that the robot is colliding with itself in one or more locations. To observe where this collision is happening, observe the timeline, which should highlight sections of the trajectory where the collisions occur. To address this issue, consider one or more of the following changes:
- Adding waypoints to the trajectory to try and avoid the collision point.
- Moving existing waypoints and locations to move the robot in a way that avoids the collision point.
- Switching trajectory types (IK or Joint)
`;

const collisionNearEnvironmentDoc = `During this action, it appears that the robot is close to colliding with an object in the environment. While this doesn't require changes, it is good practice to avoid these scenarios, as future issues may arise. To observe where this near collision is happening, observe the timeline, which should highlight sections of the trajectory where the issue occurs. To address this issue, consider one or more of the following changes:
- Adding waypoints to the trajectory to try and move away from the near collision point
- Moving existing waypoints and locations to move the robot in a way that avoids the near collision point.
- Switching trajectory types (IK or Joint)
`;

const collisionEnvironmentDoc = `During this action, it appears that the robot is colliding with an object in the environment in one or more locations. To observe where this collision is happening, observe the timeline, which should highlight sections of the trajectory where the collisions occur. To address this issue, consider one or more of the following changes:
- Adding waypoints to the trajectory to try and avoid the collision point.
- Moving existing waypoints and locations to move the robot in a way that avoids the collision point.
- Switching trajectory types (IK or Joint)
`;

const occupancyNearDoc = `During this action, it appears that the robot is moving near to an area designated as frequently occupied by people. While this doesn't require changes, it is good practice to avoid these scenarios, as future issues may arise. To observe where this near collision is happening, observe the timeline, which should highlight sections of the trajectory where the issue occurs. To address this issue, consider one or more of the following changes:
- Adding waypoints to the trajectory to try and move away from the occupancy zone.
- Moving existing waypoints and locations to move the robot in a way that moves further away from the occupancy zone.
- Switching trajectory types (IK or Joint)
`;

const occupancyDoc = `During this action, it appears that the robot is entering an area designated as frequently occupied by people. This can result in undesirable surprises and injuries to any individuals working in those areas. To observe where this near collision is happening, observe the timeline, which should highlight sections of the trajectory where the issue occurs. To address this issue, consider one or more of the following changes:
- Adding waypoints to the trajectory to try and move away from the occupancy zone.
- Moving existing waypoints and locations to move the robot in a way that moves further away from the occupancy zone.
- Switching trajectory types (IK or Joint)
`;

const pinchPointDoc = `During this action, it appears that the robot's movement creates areas that would be dangerous for humans to be around. Specificially, pinch points results in areas that would be unsafe for a human hand to be in, as it could pinch and subsequently hurt the individual's hand. To address this issue, consider one or more of the following changes:
- Adding waypoints to move the robot in a way that avoids the pinch point.
- Moving existing waypoints and locations to move the robot in a way that avoids that pinch point.
`;

const thingSafetyDoc = `During this action, it appears that the robot is trying to grasp an unsafe object. Moving the robot while it's holding an unsafe object creates a dangerous work environment for any people working near to or with the robot. To address this issue, consider one or more of the following changes:
- Placing the item in a safe to carry container before moving it about.
`;

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
    let initialTimeData = {};
    sceneUpdates.forEach(step => {
        if (step.source && moveTrajectoryIDs.includes(step.source)) {
            if (!(step.source in timeData)) {
                timeData[step.source] = [];
                initialTimeData[step.source] = step.time;
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
            graphData.push({x: timeData[moveID][i] - initialTimeData[moveID], endEffectorScore: scores[i]});
        }

        if (shouldReportIssue) {
            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
                requiresChanges: hasError,
                title: `End effector pose is poor`,
                description: `End effector pose is poor`,
                featuredDocs: {[moveID]: endEffectorPoseDoc},
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
                    decimals: 5,
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

    // Build pairing of links to collision objects
    let envrionTracker = [];
    const collisionIDs = lodash.filter(programData, function (v) { return v.type === 'collisionShapeType' }).map(collision => { return collision.id });
    robotJointIDs.forEach(rID => {
        collisionIDs.forEach(fID => {
            envrionTracker.push({link1: rID, link2: fID});
        })
    });

    let timeData = {};
    let initialTimeData = {};
    let sCol = {};
    let eCol = {};
    let positionData = {};
    let moveTrajectoryIDs = [];

    compiledData[program.id]?.[ROOT_PATH]?.steps?.forEach(step => {
        if (step.type === STEP_TYPE.SCENE_UPDATE && programData[step.source]?.type === "moveTrajectoryType") {
            if (!timeData[step.source]) {
                moveTrajectoryIDs.push(step.source);
                timeData[step.source] = [];
                initialTimeData[step.source] = step.time;
                sCol[step.source] = [];
                eCol[step.source] = [];
                positionData[step.source] = [];
            }
            timeData[step.source].push(step.time);
            positionData[step.source].push({...step.data.links});

            let sColAdjusted = {}
            let eColAdjusted = {}
            robotJointIDs.forEach(jid => {
                sColAdjusted[jid] = Number.MAX_VALUE;
                eColAdjusted[jid] = Number.MAX_VALUE;
            });

            step.data.proximity.forEach(({ shape1, shape2, distance, points, physical }) => {
                let r1 = robotJointIDs.includes(shape1);
                let r2 = robotJointIDs.includes(shape2);
                if (!(r1 && r2)) {
                    let link = r1 ? shape1 : shape2;
                    eColAdjusted[link] = Math.min(eColAdjusted[link], distance);
                } else {
                    sColAdjusted[shape1] = Math.min(sColAdjusted[shape1], distance);
                    sColAdjusted[shape2] = Math.min(sColAdjusted[shape2], distance);
                }
            });
            sCol[step.source].push(sColAdjusted);
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
            let timestamp = timeData[moveID][i] - initialTimeData[moveID];
            graphData[selfIndex].push({x: timestamp});
            graphData[envIndex].push({x: timestamp});

            Object.keys(sCol[moveID][i]).forEach(rLink => {
                if (positionData[moveID][i][rLink]) {
                    if (typeof(positionData[moveID][i][rLink].rotation?.w) === typeof(1)) {
                        environmentModel = updateEnvironModelQuaternion(environmentModel, rLink, positionData[moveID][i][rLink].position, positionData[moveID][i][rLink].rotation)
                    } else {
                        environmentModel = updateEnvironModel(environmentModel, rLink, positionData[moveID][i][rLink].position, positionData[moveID][i][rLink].rotation)
                    }
                } else {
                    console.log('ERROR, rLink, POSITION', rLink, positionData[moveID][i][rLink]);
                }
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
                featuredDocs: {[moveID]: collisionErrors[selfIndex] ? collisionSelfDoc : collisionNearSelfDoc},
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
                    decimals: 5,
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
                requiresChanges: collisionErrors[envIndex],
                title: collisionErrors[envIndex] ? `Robot collides with environment` : `Robot is in near collision with environment`,
                description: `Robot collides with the environment`,
                featuredDocs: {[moveID]: collisionErrors[envIndex] ? collisionEnvironmentDoc : collisionNearEnvironmentDoc},
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
                    decimals: 5,
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
    let initialTimeData = {};
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
                initialTimeData[step.source] = step.time;
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
        let warningZone = false;
        let filteredGraphData = [];


        for (let j = 0; j < proximityData[source].length; j++) {
            let dataPoint = {x: timeData[source][j] - initialTimeData[source]};
            for (let i = 0; i < shouldGraphlink[source].length; i++) {
                // Grab the position data for each link and mark it's color
                if (typeof(positionData[source][j][linkNames[i]].rotation?.w) === typeof(1)) {
                    environmentModel = updateEnvironModelQuaternion(environmentModel, linkNames[i], positionData[source][j][linkNames[i]].position, positionData[source][j][linkNames[i]].rotation)
                } else {
                    environmentModel = updateEnvironModel(environmentModel, linkNames[i], positionData[source][j][linkNames[i]].position, positionData[source][j][linkNames[i]].rotation)
                }
                
                const posRot = queryWorldPose(environmentModel, linkNames[i], '');
                let curFrame = posRot.position;
                let point = null;

                if (proximityData[source][j][linkNames[i]]) {
                    Object.values(proximityData[source][j][linkNames[i]]).forEach(obj => {
                        if (obj.distance <= errorLevel) {
                            enteredZone = true;
                            warningZone = true;
                            occupancyValues[source][linkNames[i]].push({position: {...curFrame}, color: hexToRgb(frameStyles.errorColors["safety"])});
                        } else if (obj.distance <= warningLevel) {
                            warningZone = true;
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

        if (warningZone) {
            const id = generateUuid('issue');
            issues[id] = {
                id: id,
                requiresChanges: false,
                title: enteredZone ? `Entered Occupancy Zone`: `Close to Occupancy Zone`,
                description: enteredZone ? `Robot trajectory entered occupancy zone`: `Robot trajectory results in close proximity to the occupancy zone`,
                featuredDocs: {[source]: enteredZone ? occupancyDoc : occupancyNearDoc},
                complete: false,
                focus: [source],
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
                    decimals: 5,
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
    let initialTimeData = {};
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
                initialTimeData[step.source] = step.time;
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
                return {x: element - initialTimeData[source], inPinch: (pinchPoints[source][idx] ? 1 : 0)}
            });

            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
                requiresChanges: true,
                title: `Likely pinch points`,
                description: `Robot trajectory includes likely pinch points`,
                featuredDocs: {[source]: pinchPointDoc},
                complete: false,
                focus: [source],
                graphData: {
                    series: graphData,
                    xAxisLabel: 'Timestamp',
                    yAxisLabel: 'Pinch Points Exist',
                    units: '',
                    thresholds: [
                        {range: ["MIN", 1], color: 'grey', label: 'OK'},
                        {range: [1, "MAX"], color: frameStyles.errorColors["safety"], label: 'Error'}
                    ],
                    decimals: 1,
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
                        featuredDocs: {[source.id]: thingSafetyDoc},
                        complete: false,
                        focus: [source.id],
                        graphData: null
                    }
                }
        }
    });

    return [issues, {}];
}
