import { generateUuid } from "../generateUuid";
import { PINCH_POINT_FIELDS, objectMap } from "../helpers";

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


export const findEndEffectorPoseIssues = ({program}) => { // Requires trace pose information
    let issues = {};

    let addressed = [];

    let warningLevel = 2;
    let errorLevel = 5;

    Object.values(program.executablePrimitives).forEach(ePrim => {
        if (ePrim) {
            Object.values(ePrim).forEach(primitive=>{
                if (primitive.type === "node.primitive.move-trajectory." && !addressed.includes(primitive.uuid)) {
                    let trajectory = primitive.parameters.trajectory_uuid;
                    let frames = trajectory.trace.frames.tool0;
                    let scores = primitive.parameters.trajectory_uuid.eePoseScores;
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
                        uuid: uuid,
                        requiresChanges: hasError,
                        title: `End effector pose is poor`,
                        description: `End effector pose is poor`,
                        complete: false,
                        focus: {uuid:primitive.uuid, type:'primitive'},
                        graphData: {
                            series: graphData,
                            lineColors: ["#CC79A7"],
                            xAxisLabel: 'Timestamp',
                            yAxisLabel: 'Pose Score',
                            title: ''
                        },
                        sceneData: {vertices: {endEffectorPose: endEffectorScores}}
                    }
                    addressed.push(primitive.uuid)
                }
            });
        }
    });

    return [issues, {}];
}

// Requires collision graders
export const findCollisionIssues = ({program}) => { 
    let issues = {};
    let addressed = [];
    const warningLevel = 0.1;
    const errorLevel = 1;

    Object.values(program.executablePrimitives).forEach(ePrim => {
        if (ePrim) {
            Object.values(ePrim).forEach(primitive=>{
                if (primitive.type === "node.primitive.move-trajectory." && !addressed.includes(primitive.uuid)) {
                    let trajectory = primitive.parameters.trajectory_uuid;
                    let timeData = trajectory.trace.time_data;
                    let sCol = trajectory.trace.self_collisions;
                    let eCol = trajectory.trace.env_collisions;
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
                            uuid: uuid,
                            requiresChanges: collisionErrors[selfIndex],
                            title: `Robot collides with self`,
                            description: `Robot collides with self`,
                            complete: false,
                            focus: {uuid:primitive.uuid, type:'primitive'},
                            graphData: {
                                series: graphData[selfIndex],
                                lineColors: linkColors[selfIndex],
                                xAxisLabel: 'Timestamp',
                                yAxisLabel: 'Proximity',
                                title: ''},
                            sceneData: {vertices: collisionData[selfIndex]}
                        }
                        addressed.push(primitive.uuid)
                    }
    
                    // Build issue for environmental collisions
                    if (graphData[envIndex].length > 0) {
                        const uuid = generateUuid('issue');
                        issues[uuid] = {
                            uuid: uuid,
                            requiresChanges: collisionErrors[selfIndex],
                            title: `Robot collides with the environment`,
                            description: `Robot collides with the environment`,
                            complete: false,
                            focus: {uuid:primitive.uuid, type:'primitive'},
                            graphData: {
                                series: graphData[envIndex],
                                lineColors: linkColors[envIndex],
                                xAxisLabel: 'Timestamp',
                                yAxisLabel: 'Proximity',
                                title: ''},
                            sceneData: {vertices: collisionData[envIndex]}
                        }
                        addressed.push(primitive.uuid)
                    }
                }
            });
        }
        
    });

    return [issues, {}];
}

// Requires occupancy zone graders
export const findOccupancyIssues = ({program}) => {
    let issues = {};

    const warningLevel = 0.8;
    const errorLevel = 1;


    Object.values(program.executablePrimitives).forEach(ePrim => {
        Object.values(ePrim).forEach(primitive=>{
            if (primitive.type === "node.primitive.move-trajectory.") {
                let trajectory = primitive.parameters.trajectory_uuid;
                let occupancyValues = {};
                let occupancy = trajectory.trace.occupancy;
                let timeData = trajectory.trace.time_data;

                let enteredZone = false;

                let shouldGraphlink = [];
                for (let i = 0; i < linkNames.length; i++) {
                    shouldGraphlink.push(false);
                    occupancyValues[linkNames[i]] = [];
                }

                for (let i = 0; i < linkNames.length; i++) {
                    for (let j = 0; j < occupancy[linkNames[i]].length; j++) {
                        if (occupancy[linkNames[i]][j] > warningLevel) {
                            shouldGraphlink[i] = true;
                            j = occupancy[linkNames[i]].length;
                        }
                    }
                }

                let filteredGraphData = [];
                for (let j = 0; j < occupancy[linkNames[0]].length; j++) {
                    let dataPoint = {x: Math.floor(timeData[j] * precision) / precision};
                    for (let i = 0; i < shouldGraphlink.length; i++) {
                        // Grab the position data for each link and mark it's color
                        let curFrame = trajectory.trace.frames[linkNames[i]][j][0];
                        if (occupancy[linkNames[i]][j] >= errorLevel) {
                            enteredZone = true;
                            occupancyValues[linkNames[i]].push({position: {x: curFrame[0], y: curFrame[1], z: curFrame[2]}, color: ERROR_COLOR});
                        } else if (occupancy[linkNames[i]][j] >= warningLevel) {
                            occupancyValues[linkNames[i]].push({position: {x: curFrame[0], y: curFrame[1], z: curFrame[2]}, color: WARNING_COLOR});
                        } else {
                            occupancyValues[linkNames[i]].push({position: {x: curFrame[0], y: curFrame[1], z: curFrame[2]}, color: NO_ERROR_COLOR});
                        }

                        // Add the data for the contextual info graph
                        if (shouldGraphlink[i]) {
                            dataPoint[linkNameMap[linkNames[i]]] = occupancy[linkNames[i]][j];
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
                        uuid: uuid,
                        requiresChanges: false,
                        title: enteredZone ? `Entered Occupancy Zone`: `Close to Occupancy Zone`,
                        description: enteredZone ? `Robot trajectory entered occupancy zone`: `Robot trajectory results in close proximity to the occupancy zone`,
                        complete: false,
                        focus: {uuid:primitive.uuid, type:'primitive'},
                        graphData: {
                            series: filteredGraphData,
                            lineColors: linkColors,
                            xAxisLabel: 'Timestamp',
                            yAxisLabel: 'Proximity',
                            title: ''},
                        sceneData: {vertices: occupancyValues}
                    }
                }
            }
        })
    });

    return [issues, {}];
}

export const findPinchPointIssues = ({unrolled}) => { // Requires pinch-point graders
    
    let issues = {};
    let addressed = [];

    // cancel if the unrolled program is invalid
    if (!unrolled) {return [issues, {}];}

    Object.values(unrolled).forEach(primitive => {
        if (primitive.type === "node.primitive.move-trajectory." && !addressed.includes(primitive.uuid)) {
            let trajectory = primitive.parameters.trajectory_uuid;
            // let timeData = trajectory.trace.time_data;
            console.log(trajectory.trace)
            const hasError = objectMap(PINCH_POINT_FIELDS,(_,field)=>trajectory.trace.pinch_points[field].some(val=>val!==null));
            console.log(hasError)
            if (Object.values(hasError).includes(true)) {
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    uuid: uuid,
                    requiresChanges: true,
                    title: `Likely pinch points`,
                    description: `Robot trajectory includes likely pinch points`,
                    complete: false,
                    focus: {uuid:primitive.uuid, type:'primitive'},
                    graphData: null,
                    sceneData: null
                }
                addressed.push(primitive.uuid)
            }
        }
    });

    return [issues, {}];
}

export const findThingMovementIssues = ({program,unrolled}) => { // May require trace pose information
    let issues = {};
    
    if (!unrolled) {
        return [issues, {}];
    }

    Object.values(unrolled).forEach(primitive=>{
        
       // console.log(primitive.type);
        if (primitive.type === 'node.primitive.gripper.' && primitive.parameters.semantic === 'grasping'){
            const thingUUID = primitive.parameters.thing_uuid.uuid;
            //console.log(thingUUID + "  123" );
            Object.values(program.data.placeholders).forEach(placeholder => {
                //console.log(placeholder.uuid );
                if (placeholder.uuid === thingUUID){
                  const thingTypeUUID = placeholder.pending_node.thing_type_uuid;
                  //console.log("true");
                  Object.values(program.data.thingTypes).forEach(thingType => {
                    //console.log(thingType.uuid + "   " + thingTypeUUID);
                      if(thingType.uuid === thingTypeUUID && thingType.is_safe === false){
                        
                        const uuid = generateUuid('issue');
                        issues[uuid] = {
                        uuid: uuid,
                        requiresChanges: true,
                        title: `Grasping on unsafe object`, // might need some changes
                        description: `The robot is grasping on a dangerous item`,// might need some changes
                        complete: false,
                        focus: {uuid:primitive.uuid, type:'primitive'},
                        graphData: null
            }
                      }else{
                          //console.log("false");
                      }
                })
                }
            })
            
           
            
        }
    })


    return [issues, {}];
}