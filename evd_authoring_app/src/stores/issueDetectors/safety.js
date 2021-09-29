import { generateUuid } from "../generateUuid";
export const findEndEffectorPoseIssues = (_) => { // Requires trace pose information
    let issues = {};

    return [issues, {}];
}

export const findCollisionIssues = (_) => { // Requires collision graders
    let issues = {};

    return [issues, {}];
}

// Requires occupancy zone graders
export const findOccupancyIssues = ({program}) => {
    let issues = {};

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

    const warningLevel = 0.8;
    const errorLevel = 1;

    // Used for adjusting the x axes time data
    const precision = 1000;

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
                        if (occupancy[linkNames[i]][j] > errorLevel) {
                            enteredZone = true;
                            occupancyValues[linkNames[i]].push({position: {x: curFrame[0], y: curFrame[1], z: curFrame[2]}, color: ERROR_COLOR});
                        } else if (occupancy[linkNames[i]][j] > warningLevel) {
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

export const findPinchPointIssues = (_) => { // Requires pinch-point graders
    let issues = {};

    return [issues, {}];
}

export const findThingMovementIssues = ({program,unrolled}) => { // May require trace pose information
    let issues = {};
    
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