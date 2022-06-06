import { STATUS, STEP_TYPE } from "../Constants";
import { generateUuid } from "../generateUuid";
import { checkHandThresholds, framesToEEPoseScores, likProximityAdjustment } from "../helpers";

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
            let trajectory = primitive.properties.trajectory;

            let steps = primitive.properties.compiled["{}"].steps.filter(v => v.type === STEP_TYPE.SCENE_UPDATE);
            let toolFrames = steps.map(step => { return step.data.links.tool0.position });
            let frames = trajectory.trace.frames.tool0;

            // let scores = primitive.parameters.trajectory_uuid.eePoseScores;
            let scores = framesToEEPoseScores(toolFrames);

            let timeData = primitive.parameters.trajectory_uuid.trace.time_data;

            let endEffectorScores = [];
            let graphData = [];
            let hasError = false;

            for (let i = 0; i < frames.length; i++) {
                let curFrame = frames[i][0]
                if (scores[i] >= errorLevel) {
                    hasError = true;
                    endEffectorScores.push({position: {x: curFrame[0], y: curFrame[1], z: curFrame[2]}, color: ERROR_COLOR});
                } else if (scores[i] >= warningLevel) {
                    endEffectorScores.push({position: {x: curFrame[0], y: curFrame[1], z: curFrame[2]}, color: WARNING_COLOR});
                } else {
                    endEffectorScores.push({position: {x: curFrame[0], y: curFrame[1], z: curFrame[2]}, color: NO_ERROR_COLOR});
                }
                graphData.push({x: Math.floor(timeData[i] * precision) / precision, endEffectorScore: scores[i]});
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
                    lineColors: ["#CC79A7"],
                    xAxisLabel: 'Timestamp',
                    yAxisLabel: 'Pose Score',
                    title: ''
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
            let steps = primitive.properties.compiled["{}"].steps.filter(v => v.type === STEP_TYPE.SCENE_UPDATE);
            let robotAgent = programData.filter(v => v.type === 'robotAgentType')[0];
            
            // Build the arrays for time
            let timeData = [];
            let sCol = {};
            let eCol = {};

            for (let i = 0; i < steps.length; i++) {
                timeData.push(steps[i].time);
                let tmp = likProximityAdjustment(robotAgent, steps[i].data.proximity)
                Object.keys(tmp).forEach(key => {
                    if (!(key in sCol)) {
                        sCol[key] = {};
                    }
                    Object.keys(tmp[key]).forEach(key2 => {
                        if (!(key2 in sCol[key])) {
                            sCol[key][key2] = [];
                        }
                        sCol[key][key2].push(temp[key][key2].distance);
                    })
                })
                // eCol.push(likProximityAdjustment())
            }

            let trajectory = primitive.parameters.trajectory_uuid;
            
            
            // let sCol = trajectory.trace.self_collisions;
            // let eCol = trajectory.trace.env_collisions;


            const selfIndex = 0;
            const envIndex = 1;
            let collisionData = [{}, {}];
            let graphData = [[], []];
            let iterationLength = Object.values(sCol[linkNames[0]]).length;
            let collisionErrors = [false, false];

            let allSelfCollisions = {};
            let allEnvCollisions = {};
            let shouldGraphlink = [[], []];
            for (let i = 0; i < linkNames.length; i++) {
                shouldGraphlink[selfIndex].push(false);
                shouldGraphlink[envIndex].push(false);
                collisionData[selfIndex][linkNames[i]] = [];
                collisionData[envIndex][linkNames[i]] = [];
                allSelfCollisions[linkNames[i]] = Object.values(sCol[linkNames[i]]);
                allEnvCollisions[linkNames[i]] = Object.values(eCol[linkNames[i]]);
            }

            // Determine what to graph/render in scene
            for (let i = 0; i < linkNames.length; i++) {
                let selfCollisions = allSelfCollisions[linkNames[i]];
                let envCollisions = allEnvCollisions[linkNames[i]];

                for (let j = 0; j < iterationLength; j++) {
                    if (selfCollisions[i] >= warningLevel) {
                        shouldGraphlink[selfIndex][i] = true;
                    }
                    if (envCollisions[i] >= warningLevel) {
                        shouldGraphlink[envIndex][i] = true;
                    }

                    if (shouldGraphlink[selfIndex][i] && shouldGraphlink[envIndex][i]) {
                        j = selfCollisions.length;
                    }
                }
            }

            // Iteratively build the graph and scene data
            for (let i = 0; i < iterationLength; i++) {
                let timestamp = Math.floor(timeData[i] * precision) / precision;
                graphData[selfIndex].push({x: timestamp});
                graphData[envIndex].push({x: timestamp});
                for (let j = 0; j < linkNames.length; j++) {
                    if (shouldGraphlink[selfIndex][j]) {
                        let curFrame = trajectory.trace.frames[linkNames[j]][i][0];

                        if (allSelfCollisions[linkNames[j]][i] >= errorLevel) {
                            collisionErrors[selfIndex] = true;
                            console.log({value:allSelfCollisions[linkNames[j]][i],link:linkNames[j]})
                            collisionData[selfIndex][linkNames[j]].push({position: {x: curFrame[0], y: curFrame[1], z: curFrame[2]}, color: ERROR_COLOR});
                        } else if (allSelfCollisions[linkNames[j]][i] >= warningLevel) {
                            collisionData[selfIndex][linkNames[j]].push({position: {x: curFrame[0], y: curFrame[1], z: curFrame[2]}, color: WARNING_COLOR});
                        } else {
                            collisionData[selfIndex][linkNames[j]].push({position: {x: curFrame[0], y: curFrame[1], z: curFrame[2]}, color: NO_ERROR_COLOR});
                        }
                        graphData[selfIndex][i][linkNameMap[linkNames[j]]] = allSelfCollisions[linkNames[j]][i];
                    }

                    if (shouldGraphlink[envIndex][j]) {
                        let curFrame = trajectory.trace.frames[linkNames[j]][i][0];

                        if (allEnvCollisions[linkNames[j]][i] >= errorLevel) {
                            collisionErrors[envIndex] = true;
                            collisionData[envIndex][linkNames[j]].push({position: {x: curFrame[0], y: curFrame[1], z: curFrame[2]}, color: ERROR_COLOR});
                        } else if (allSelfCollisions[linkNames[j]][i] >= warningLevel) {
                            collisionData[envIndex][linkNames[j]].push({position: {x: curFrame[0], y: curFrame[1], z: curFrame[2]}, color: WARNING_COLOR});
                        } else {
                            collisionData[envIndex][linkNames[j]].push({position: {x: curFrame[0], y: curFrame[1], z: curFrame[2]}, color: NO_ERROR_COLOR});
                        }
                        graphData[envIndex][i][linkNameMap[linkNames[j]]] = allEnvCollisions[linkNames[j]][i];
                    }
                }
            }

            // Build colors for the graph
            let linkColors = [[], []];
            for (let i = 0; i < linkNames.length; i++) {
                if (shouldGraphlink[selfIndex][i]) {
                    linkColors[selfIndex].push(linkColorMap[linkNames[i]]);
                }
                if (shouldGraphlink[envIndex][i]) {
                    linkColors[envIndex].push(linkColorMap[linkNames[i]]);
                }
            }

            // Build issue for self collisions
            if (graphData[selfIndex].length > 0) {
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
                        lineColors: linkColors[selfIndex],
                        xAxisLabel: 'Timestamp',
                        yAxisLabel: 'Proximity',
                        title: ''},
                    sceneData: {vertices: collisionData[selfIndex]}
                }
                addressed.push(primitive.id)
            }

            // Build issue for environmental collisions
            if (graphData[envIndex].length > 0) {
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
                        lineColors: linkColors[envIndex],
                        xAxisLabel: 'Timestamp',
                        yAxisLabel: 'Proximity',
                        title: ''},
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
        let steps = primitive.properties.compiled["{}"].steps.filter(v => v.type === STEP_TYPE.SCENE_UPDATE);
        
        // Build the arrays for time
        let timeData = [];
        let proximityData = [];
        for (let i = 0; i < steps.length; i++) {
            timeData.push(steps[i].time);
            proximityData.push(likProximityAdjustment(null, steps[i].data.proximity.filter(v => !v.physical)));
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
                    lineColors: linkColors,
                    xAxisLabel: 'Timestamp',
                    yAxisLabel: 'Proximity',
                    title: '',
                    isTimeseries: false
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