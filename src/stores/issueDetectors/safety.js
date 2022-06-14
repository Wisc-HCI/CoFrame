import frameStyles from "../../frameStyles";
import { STATUS, STEP_TYPE } from "../Constants";
import { generateUuid } from "../generateUuid";
import { checkHandThresholds, stepsToEEPoseScores, likProximityAdjustment } from "../helpers";
import lodash from 'lodash';

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

// Used for adjusting the x axes time data
const precision = 1000;

export const findEndEffectorPoseIssues = ({programData, settings}) => { // Requires trace pose information
    let issues = {};

    let addressed = [];

    let warningLevel = settings['eePoseWarn'].value;
    let errorLevel = settings['eePoseErr'].value;

    Object.values(programData).filter(v => v.type === 'moveTrajectoryType').forEach(primitive => {
        if (!addressed.includes(primitive.id)) {
            // let trajectory = primitive.properties.trajectory;

            let steps = primitive.properties.compiled["{}"] ? primitive.properties.compiled["{}"].steps.filter(v => v.type === STEP_TYPE.SCENE_UPDATE) : [];
            let toolFrames = [];
            let timeData = [];
            for (let i = 0; i < steps.length; i++) {
                toolFrames.push(steps[i].data.links.tool0.position);
                timeData.push(steps[i].time);
            }

            let gripper = lodash.filter(programData, function (v) { return v.type === 'gripperType'})[0];
            let scores = stepsToEEPoseScores(toolFrames, gripper);

            let endEffectorScores = [];
            let graphData = [];
            let hasError = false;

            for (let i = 0; i < toolFrames.length; i++) {
                let curFrame = toolFrames[i]
                if (scores[i] >= errorLevel) {
                    hasError = true;
                    endEffectorScores.push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: ERROR_COLOR});
                } else if (scores[i] >= warningLevel) {
                    endEffectorScores.push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: WARNING_COLOR});
                } else {
                    endEffectorScores.push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: NO_ERROR_COLOR});
                }
                graphData.push({x: timeData[i] / 1000, endEffectorScore: scores[i]});
            }

            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
                requiresChanges: hasError,
                title: `End effector pose is poor`,
                description: `End effector pose is poor`,
                complete: false,
                focus: [primitive.id],
                graphData: {
                    series: graphData,
                    xAxisLabel: 'Timestamp',
                    yAxisLabel: 'Pose Score',
                    title: '',
                    warningThreshold: warningLevel,
                    errorThreshold: errorLevel,
                    warningColor: frameStyles.colors["safety"],
                    errorColor: frameStyles.errorColors["safety"],
                    isTimeseries: true
                },
                sceneData: {vertices: {endEffectorPose: endEffectorScores}}
            }
            addressed.push(primitive.id)
        }
    });

    return [issues, {}];
}

// Requires collision graders
export const findCollisionIssues = ({programData, settings}) => { 
    let issues = {};
    let addressed = [];
    const warningLevel = settings['collisionWarn'].value;
    const errorLevel = settings['collisionErr'].value;
    
    Object.values(programData).filter(v => v.type === 'moveTrajectoryType').forEach(primitive => {
        if (!addressed.includes(primitive.id)) {
            let steps = primitive.properties.compiled["{}"] ? primitive.properties.compiled["{}"].steps.filter(v => v.type === STEP_TYPE.SCENE_UPDATE) : [];
            let robotAgent = lodash.filter(programData, function (v) { return v.type === 'robotAgentType'})[0];
            let robotPoints = robotAgent ? robotAgent.properties.pinchPointPairLinks : [];
            
            // Build the arrays for time
            let timeData = [];
            let sCol = [];
            let eCol = [];


            // Build pairing for environment tracking
            let fixtureIDs = lodash.filter(programData, function (v) { return v.type === 'fixtureType'}).map(fixture => { return fixture.id });
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

            for (let i = 0; i < steps.length; i++) {
                timeData.push(steps[i].time);

                // Self Collisions
                let sColTmp = likProximityAdjustment(robotPoints, steps[i].data.proximity, true);
                let sColAdjusted = {}
                // For each joint, assign it the minimum proximity value
                Object.keys(sColTmp).forEach(key => {
                    if (linkNames.includes(key)) {
                        sColAdjusted[key] = Math.min(...Object.values(sColTmp[key]).map(v => {return v.distance}));
                    }
                })
                sCol.push(sColAdjusted);

                // Environment Collisions
                let eColTmp = likProximityAdjustment(envrionTracker, steps[i].data.proximity, true);
                let eColAdjusted = {}
                // For each joint, assign it the minimum proximity value
                Object.keys(eColTmp).forEach(key => {
                    if (linkNames.includes(key)) {
                        eColAdjusted[key] = Math.min(...Object.values(eColTmp[key]).map(v => {return v.distance}));
                    }
                })
                eCol.push(eColAdjusted);
            }


            const selfIndex = 0;
            const envIndex = 1;
            let collisionErrors = [false, false];

            // Iteratively build the graph and scene data
            let graphData = [[], []];
            let collisionData = [{}, {}];
            let shouldGraphSet = [false, false];
            let shouldGraphJoint = [{}, {}];

            // For each joint (at each timestep), determine if it's within the warning threshold
            for (let i = 0; i < timeData.length; i++) {
                Object.keys(sCol[i]).forEach(rLink => {
                    if (!(rLink in shouldGraphJoint[selfIndex])) {
                        shouldGraphJoint[selfIndex][rLink] = false;
                    }

                    if (sCol[i][rLink] <= warningLevel) {
                        shouldGraphJoint[selfIndex][rLink] = true;
                    }
                });

                Object.keys(eCol[i]).forEach(rLink => {
                    if (!(rLink in shouldGraphJoint[envIndex])) {
                        shouldGraphJoint[envIndex][rLink] = {};
                    }

                    if (eCol[i][rLink] <= warningLevel) {
                        shouldGraphJoint[envIndex][rLink] = true;
                    }
                });
            }


            // Build the graph and collision data
            // If a joint should be graphed, add it's data points to the respective array
            for (let i = 0; i < timeData.length; i++) {
                let timestamp = timeData[i] / 1000;
                graphData[selfIndex].push({x: timestamp});
                graphData[envIndex].push({x: timestamp});

                Object.keys(sCol[i]).forEach(rLink => {
                    if (shouldGraphJoint[selfIndex][rLink]) {
                        let curFrame = steps[i].data.links[rLink].position;
                        shouldGraphSet[selfIndex] = true;

                        if (!collisionData[selfIndex][rLink]) {
                            collisionData[selfIndex][rLink] = [];
                        }

                        if (sCol[i][rLink] <= errorLevel) {
                            collisionErrors[selfIndex] = true;
                            collisionData[selfIndex][rLink].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: ERROR_COLOR});
                        } else if (sCol[i][rLink] <= warningLevel) {
                            collisionData[selfIndex][rLink].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: WARNING_COLOR});
                        } else {
                            collisionData[selfIndex][rLink].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: NO_ERROR_COLOR});
                        }
                        graphData[selfIndex][i][linkNameMap[rLink]] = sCol[i][rLink];
                    }
                });

                Object.keys(eCol[i]).forEach(rLink => {
                    if (shouldGraphJoint[envIndex][rLink]) {
                        let curFrame = steps[i].data.links[rLink].position;
                        shouldGraphSet[envIndex] = true;

                        if (!collisionData[envIndex][rLink]) {
                            collisionData[envIndex][rLink] = [];
                        }

                        if (eCol[i][rLink] <= errorLevel) {
                            collisionErrors[envIndex] = true;
                            collisionData[envIndex][rLink].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: ERROR_COLOR});
                        } else if (eCol[i][rLink] <= warningLevel) {
                            collisionData[envIndex][rLink].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: WARNING_COLOR});
                        } else {
                            collisionData[envIndex][rLink].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: NO_ERROR_COLOR});
                        }
                        graphData[envIndex][i][linkNameMap[rLink]] = eCol[i][rLink];
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
                    focus: [primitive.id],
                    graphData: {
                        series: graphData[selfIndex],
                        xAxisLabel: 'Timestamp',
                        yAxisLabel: 'Proximity',
                        title: '',
                        warningThreshold: warningLevel,
                        errorThreshold: errorLevel,
                        warningColor: frameStyles.colors["safety"],
                        errorColor: frameStyles.errorColors["safety"],
                        isTimeseries: true
                    },
                    sceneData: {vertices: collisionData[selfIndex]}
                }
                addressed.push(primitive.id)
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
                    focus: [primitive.id],
                    graphData: {
                        series: graphData[envIndex],
                        xAxisLabel: 'Timestamp',
                        yAxisLabel: 'Proximity',
                        title: '',
                        warningThreshold: warningLevel,
                        errorThreshold: errorLevel,
                        warningColor: frameStyles.colors["safety"],
                        errorColor: frameStyles.errorColors["safety"],
                        isTimeseries: true
                    },
                    sceneData: {vertices: collisionData[envIndex]}
                }
                addressed.push(primitive.id)
            }
        }
    });

    return [issues, {}];
}

// Requires occupancy zone graders
export const findOccupancyIssues = ({programData, settings}) => {
    let issues = {};

    const warningLevel = settings['occupancyWarn'].value;
    const errorLevel = settings['occupancyErr'].value;

    Object.values(programData).filter(v => v.type === "moveTrajectoryType").forEach(primitive => {
        let steps = primitive.properties.compiled["{}"] ? primitive.properties.compiled["{}"].steps.filter(v => v.type === STEP_TYPE.SCENE_UPDATE) : [];
        
        // Build the arrays for time
        let timeData = [];
        let proximityData = [];
        for (let i = 0; i < steps.length; i++) {
            timeData.push(steps[i].time);
            proximityData.push(likProximityAdjustment([], steps[i].data.proximity.filter(v => !v.physical, false)));
        }

        let occupancyValues = {};
        let enteredZone = false;

        let shouldGraphlink = [];
        for (let i = 0; i < linkNames.length; i++) {
            shouldGraphlink.push(false);
            occupancyValues[linkNames[i]] = [];
        }

        // Figure out which links to graph
        for (let i = 0; i < proximityData.length; i++) {
            for (let j = 0; j < linkNames.length; j++) {
                if (!shouldGraphlink[j] && proximityData[i][linkNames[j]]) {
                    Object.values(proximityData[i][linkNames[j]]).forEach(obj => {
                        if (obj.distance > warningLevel) {
                                shouldGraphlink[j] = true;
                                j = linkNames.length;
                        }
                    });
                }
            }
        }

        let filteredGraphData = [];
        for (let j = 0; j < proximityData.length; j++) {
            let dataPoint = {x: timeData[j]};
            for (let i = 0; i < shouldGraphlink.length; i++) {
                // Grab the position data for each link and mark it's color
                let curFrame = steps[j].data.links[linkNames[i]].position;
                let point = null;

                if (proximityData[j][linkNames[i]]) {
                    Object.values(proximityData[j][linkNames[i]]).forEach(obj => {
                        if (obj.distance > errorLevel) {
                            enteredZone = true;
                            occupancyValues[linkNames[i]].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: ERROR_COLOR});
                        } else if (obj.distance > warningLevel) {
                            occupancyValues[linkNames[i]].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: WARNING_COLOR});
                        } else {
                            occupancyValues[linkNames[i]].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: NO_ERROR_COLOR});
                        }
                        point = obj.distance;
                    });
                }

                // Add the data for the contextual info graph
                if (shouldGraphlink[i] && point !== null) {
                    dataPoint[linkNameMap[linkNames[i]]] = point;
                }
            }

            if (Object.keys(dataPoint).length > 1) {
                filteredGraphData.push(dataPoint);
            }
        }

        let linkColors = [];
        for (let i = 0; i < linkNames.length; i++) {
            if (shouldGraphlink[i]) {
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
                focus: [primitive.id],
                graphData: {
                    series: filteredGraphData,
                    xAxisLabel: 'Timestamp',
                    yAxisLabel: 'Proximity',
                    title: '',
                    warningThreshold: warningLevel,
                    errorThreshold: errorLevel,
                    warningColor: frameStyles.colors["safety"],
                    errorColor: frameStyles.errorColors["safety"],
                    isTimeseries: true
                },
                sceneData: {vertices: occupancyValues}
            }
        }
    });

    return [issues, {}];
}

export const findPinchPointIssues = ({programData, program}) => { // Requires pinch-point graders
    
    let issues = {};
    let addressed = [];

    Object.values(program.properties.children).forEach(key => {
        let primitive = programData[key];
        if (primitive.type === "moveTrajectoryType" && !addressed.includes(primitive.id)) {
            let hasError = false;
            primitive.properties.compiled["{}"].steps.forEach((step) => {
                let proxData = step.data?.proximity;
                if (proxData) {
                    proxData.forEach(entry => {
                        hasError = hasError || (entry.distance !== null && checkHandThresholds(entry.distance));
                    });
                }
            });
            if (hasError) {
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    id: uuid,
                    requiresChanges: true,
                    title: `Likely pinch points`,
                    description: `Robot trajectory includes likely pinch points`,
                    complete: false,
                    focus: [primitive.id],
                    graphData: null,
                    sceneData: null
                }
                addressed.push(primitive.id)
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