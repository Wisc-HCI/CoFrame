import { generateUuid } from "../generateUuid"

const jointNames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'];
const jointNameMap = {
    'shoulder_pan_joint': 'Shoulder Pan Joint',
    'shoulder_lift_joint': 'Shoulder Lift Joint',
    'elbow_joint': 'Elbow Joint',
    'wrist_1_joint': 'Wrist 1 Joint',
    'wrist_2_joint': 'Wrist 2 Joint',
    'wrist_3_joint': 'Wrist 3 Joint'
};
const jointLinkMap = {
    'shoulder_pan_joint': 'shoulder_link',
    'shoulder_lift_joint': 'upper_arm_link',
    'elbow_joint': 'forearm_link',
    'wrist_1_joint': 'wrist_1_link',
    'wrist_2_joint': 'wrist_2_link',
    'wrist_3_joint': 'wrist_3_link'
};
const jointThresholds = {
    'shoulder_pan_joint': {warning: 1.9, error: 2},
    'shoulder_lift_joint': {warning: 1.9, error: 2},
    'elbow_joint': {warning: 1.9, error: 2},
    'wrist_1_joint': {warning: 1.9, error: 2},
    'wrist_2_joint': {warning: 1.9, error: 2},
    'wrist_3_joint': {warning: 1.9, error: 2}
};
const jointColorMap = {
    'shoulder_pan_joint': '#009e9e',
    'shoulder_lift_joint': '#9e0000',
    'elbow_joint': '#9e0078',
    'wrist_1_joint': '#9c9e00',
    'wrist_2_joint': '#9e7100',
    'wrist_3_joint': '#0b9e00'
};

const NO_ERROR_COLOR = {r: 255, g: 255, b: 255};
const WARNING_COLOR = {r: 230, g: 159, b: 0};
const ERROR_COLOR = {r: 204, g: 75, b: 10};

// Used for adjusting the x axes time data
const precision = 1000;

export const findReachabilityIssues = ({program}) => { // requires joint_processor to produce joints for each waypoint/location
    let issues = {};
    let usedPoses = [];
    // First check through the trajectories that are used.
    Object.values(program.data.trajectories).forEach(trajectory=>{
        // Check through the start locations. If not defined, this is resolved in program quality.
        if (trajectory.start_location_uuid) {
            const startLocation = program.data.locations[trajectory.start_location_uuid];
            if (!startLocation.joints.reachable && usedPoses.indexOf(trajectory.start_location_uuid) < 0) {
                usedPoses.push(trajectory.start_location_uuid)
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    uuid: uuid,
                    requiresChanges: true,
                    title: `Location "${startLocation.name}" not reachable`,
                    description: `Location "${startLocation.name}" is not reachable by the robot, but is used in the program.`,
                    complete: false,
                    focus: {uuid:trajectory.start_location_uuid, type:'location'},
                    graphData: null
                }
            }
        };
        // Check through the end locations. If not defined, this is resolved in program quality.
        if (trajectory.end_location_uuid) {
            const endLocation = program.data.locations[trajectory.end_location_uuid];
            if (!endLocation.joints.reachable && usedPoses.indexOf(trajectory.end_location_uuid) < 0) {
                usedPoses.push(trajectory.end_location_uuid)
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    uuid: uuid,
                    requiresChanges: true,
                    title: `Location "${endLocation.name}" not reachable`,
                    description: `Location "${endLocation.name}" is not reachable by the robot, but is used in the program.`,
                    complete: false,
                    focus: {uuid:trajectory.end_location_uuid, type:'location'},
                    graphData: null
                }
            }
        }
        // Now go through the waypoints of the trajectory
        trajectory.waypoint_uuids.forEach(waypoint_uuid=>{
            const waypoint = program.data.waypoints[waypoint_uuid];
            if (!waypoint.joints.reachable && usedPoses.indexOf(waypoint_uuid) < 0) {
                usedPoses.push(waypoint_uuid)
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    uuid: uuid,
                    requiresChanges: true,
                    title: `Waypoint "${waypoint.name}" not reachable`,
                    description: `Waypoint "${waypoint.name}" is not reachable by the robot, but is used in the program.`,
                    complete: false,
                    focus: {uuid:waypoint_uuid, type:'waypoint'},
                    graphData: null
                }
            }
        })
    })

    // The initialize skill takes in a location parameter, so check for calls that use it.
    Object.values(program.data.primitives).forEach(primitive=>{
        if (primitive.type === 'node.primitive.skill-call.') {
            Object.keys(primitive.parameters).filter(key=>key.includes('location')).forEach(key=>{
                const location_uuid = primitive.parameters[key];
                console.log(location_uuid);
                if (location_uuid && program.data.locations[location_uuid].joints && !program.data.locations[location_uuid].joints.reachable) {
                    const uuid = generateUuid('issue');
                    issues[uuid] = {
                        uuid: uuid,
                        requiresChanges: true,
                        title: `Initial robot Location "${program.data.locations[location_uuid].name}" not reachable`,
                        description: `Location "${program.data.locations[location_uuid].name}" is used for initialization but is not reachable.`,
                        complete: false,
                        focus: {uuid:location_uuid, type:'location'},
                        graphData: null
                    }
                }
            })
        }
    })

    // Enumerate locations and add warnings for those not used in the program.
    Object.values(program.data.locations).forEach(location=>{
        if (!location.joints.reachable && usedPoses.indexOf(location.uuid) < 0) {
            usedPoses.push(location.uuid)
            const uuid = generateUuid('issue');
            issues[uuid] = {
                uuid: uuid,
                requiresChanges: false,
                title: `Unused Location "${location.name}" not reachable`,
                description: `Location "${location.name}" is not reachable by the robot, but isn't used in the program.`,
                complete: false,
                focus: {uuid:location.uuid, type:'location'},
                graphData: null
            }
        }
    })
    // Enumerate waypoints and add warnings for those not used in the program.
    Object.values(program.data.waypoints).forEach(waypoint=>{
        if (!waypoint.joints.reachable && usedPoses.indexOf(waypoint.uuid) < 0) {
            usedPoses.push(waypoint.uuid)
            const uuid = generateUuid('issue');
            issues[uuid] = {
                uuid: uuid,
                requiresChanges: false,
                title: `Unused Waypoint "${waypoint.name}" not reachable`,
                description: `Waypoint "${waypoint.name}" is not reachable by the robot, but isn't used in the program.`,
                complete: false,
                focus: {uuid:waypoint.uuid, type:'waypoint'},
                graphData: null
            }
        }
    })
    return [issues, {}];
}

// requires trace processor, joint speed grader (can also use intermediate type)
export const findJointSpeedIssues = ({program}) => {
    let issues = {};

    Object.values(program.executablePrimitives).forEach(ePrim => {
        Object.values(ePrim).forEach(primitive=>{
            if (primitive.type === "node.primitive.move-trajectory.") {
                let trajectory = primitive.parameters.trajectory_uuid;

                let sceneData = {};
                let jointVelocities = {};
                let jointGraphData = [];
                let timeData = trajectory.trace.time_data;
                let allJointData = trajectory.trace.joint_data;

                let jointDataLength = trajectory.trace.joint_data[jointNames[0]].length;

                let hasWarningVelocity = false;
                let hasErrorVelocity = false;
                let shouldGraphJoint = [];

                for (let i = 0; i < jointNames.length; i++) {
                    jointVelocities[jointNames[i]] = [0];
                    sceneData[jointNames[i]] = [];
                    shouldGraphJoint.push(false);
                }

                // Calculate velocites and determine which to graph
                for (let i = 0; i < jointNames.length; i++) {
                    for (let j = 1; j < jointDataLength; j++) {
                        let calcVel = Math.abs((allJointData[jointNames[i]][j] - allJointData[jointNames[i]][j-1]) / (timeData[j] - timeData[j-1]));
                        let curFrame = trajectory.trace.frames[jointLinkMap[jointNames[i]]][j][0];

                        if (calcVel > jointThresholds[jointNames[i]].error) {
                            if (!hasErrorVelocity) {
                                hasErrorVelocity = true;
                            }
                            if (!hasWarningVelocity) {
                                hasWarningVelocity = true;
                            }
                            if (!shouldGraphJoint[i]) {
                                shouldGraphJoint[i] = true;
                            }
                            sceneData[jointNames[i]].push({position: {x: curFrame[0], y: curFrame[1], z: curFrame[2]}, color: ERROR_COLOR});
                        } else if (calcVel > jointThresholds[jointNames[i]].warning) {
                            if (!hasWarningVelocity) {
                                hasWarningVelocity = true;
                            }
                            if (!shouldGraphJoint[i]) {
                                shouldGraphJoint[i] = true;
                            }
                            sceneData[jointNames[i]].push({position: {x: curFrame[0], y: curFrame[1], z: curFrame[2]}, color: WARNING_COLOR});
                        } else {
                            sceneData[jointNames[i]].push({position: {x: curFrame[0], y: curFrame[1], z: curFrame[2]}, color: NO_ERROR_COLOR});
                        }
                        jointVelocities[jointNames[i]].push(calcVel);
                    }
                }

                // Filter and format graph data
                for (let i = 1; i < jointDataLength; i++) {
                    let graphDataPoint = {x: Math.floor(timeData[i] * precision) / precision};
                    for (let j = 0; j < jointNames.length; j++) {
                        if (shouldGraphJoint[j]) {
                            graphDataPoint[jointNameMap[jointNames[j]]] = jointVelocities[jointNames[j]][i];
                        }
                    }
                    jointGraphData.push(graphDataPoint);
                }

                // Get associated colors
                let jointColors = [];
                for (let i = 0; i < jointNames.length; i++) {
                    if (shouldGraphJoint[i]) {
                        jointColors.push(jointColorMap[jointNames[i]]);
                    }
                }

                // Build issue
                if (hasWarningVelocity) {
                    const uuid = generateUuid('issue');
                    issues[uuid] = {
                        uuid: uuid,
                        requiresChanges: hasErrorVelocity,
                        title: `Robot joint(s) move too fast`,
                        description: `The robot's joint speeds are too fast`,
                        complete: false,
                        focus: {uuid:primitive.uuid, type:'primitive'},
                        graphData: {
                            series: jointGraphData,
                            lineColors: jointColors,
                            xAxisLabel: 'Timestamp',
                            yAxisLabel: 'Velocity',
                            title: ''
                        },
                        sceneData: {vertices: sceneData}
                    }
                }
            }
        })
    });

    return [issues, {}];
}

// requires trace processor, end effector grader + intermediate end effector speed interediate type
export const findEndEffectorSpeedIssues = ({program}) => {
    let issues = {};

    const warningLevel = 0.3;
    const errorLevel = 0.45;
    
    Object.values(program.executablePrimitives).forEach(ePrim => {
        Object.values(ePrim).forEach(primitive=>{
            if (primitive.type === "node.primitive.move-trajectory.") {
                let trajectory = primitive.parameters.trajectory_uuid;
                let endEffectorVelocities = [];
                let endEffectorGraphData = [];
                let timeData = trajectory.trace.time_data;
                let frames = trajectory.trace.frames.tool0;

                let hasErrorVelocity = false;
                let hasWarningVelocity = false;

                for (let i = 1; i < frames.length; i++) {
                    let calcVel = Math.sqrt(Math.pow(frames[i][0][0] - frames[i-1][0][0], 2) + Math.pow(frames[i][0][1] - frames[i-1][0][1], 2) + Math.pow(frames[i][0][2] - frames[i-1][0][2], 2)) / (timeData[i] - timeData[i-1]);

                    if (calcVel > errorLevel) {
                        if (!hasErrorVelocity) {
                            hasErrorVelocity = true;
                        }
                        if (!hasWarningVelocity) {
                            hasWarningVelocity = true;
                        }
                        endEffectorVelocities.push({position: {x: frames[i][0][0], y: frames[i][0][1], z: frames[i][0][2]}, color: ERROR_COLOR});
                    } else if (calcVel > warningLevel) {
                        if (!hasWarningVelocity) {
                            hasWarningVelocity = true;
                        }
                        endEffectorVelocities.push({position: {x: frames[i][0][0], y: frames[i][0][1], z: frames[i][0][2]}, color: WARNING_COLOR});
                    } else {
                        endEffectorVelocities.push({position: {x: frames[i][0][0], y: frames[i][0][1], z: frames[i][0][2]}, color: NO_ERROR_COLOR});
                    }

                    endEffectorGraphData.push({x: Math.floor(timeData[i] * precision) / precision, 'End Effector Velocity': calcVel});
                    
                }

                if (hasErrorVelocity) {
                    const uuid = generateUuid('issue');
                    issues[uuid] = {
                        uuid: uuid,
                        requiresChanges: hasErrorVelocity,
                        title: `End effector moves too fast`,
                        description: `The end effector moves too fast for Trajectory "${trajectory.name}"`,
                        complete: false,
                        focus: {uuid:primitive.uuid, type:'primitive'},
                        graphData: {
                            series: endEffectorGraphData,
                            lineColors: ["#E69F00"],
                            xAxisLabel: 'Timestamp',
                            yAxisLabel: 'Velocity',
                            title: ''
                        },
                        sceneData: {vertices: {endEffector: endEffectorVelocities}}
                    }
                }
            }
        })
    });

    return [issues, {}];
}

export const findPayloadIssues = (_) => { // Shouldn't change during a trajectory so more of a check on thing weight vs. robot payload (e.g., 3kg in 1g)
    let issues = {};

    return [issues, {}];
}

// Requires a convex hall operation on joint frames in traces. This volume can be compared against whole workcell (fraction) and can be used for intersection with extruded human occupancy zones 
export const findSpaceUsageIssues = ({program, stats}) => {
    let issues = {};
    let addStats = {};

    const warningLevel = 0.01;
    const errorLevel = 0.2;
    
    Object.values(program.executablePrimitives).forEach(ePrim => {
        if (ePrim) {
            Object.values(ePrim).forEach(primitive=>{
                if (primitive.type === "node.primitive.move-trajectory.") {
                    let trajectory = primitive.parameters.trajectory_uuid;
                    let spaceUtilization = trajectory.vertices;
                    
                    let isWarning = false;
                    let isError = false;

                    // get prior values
                    let priorData = [];
                    let i = 0;
                    for (i = 0; i < stats.length; i++) {
                        if (stats[i][trajectory.uuid] && stats[i][trajectory.uuid].volume) {
                            priorData.push({x:i, spaceUsage:stats[i][trajectory.uuid].volume});
                        }
                    }
                    // add new one
                    let newData = {x:i, spaceUsage:trajectory.volume}
                    priorData.push(newData);

                    // Adjust color of hull
                    let hullColor = {...NO_ERROR_COLOR, a: 0.5};
                    if (trajectory.volume > errorLevel) {
                        isError = true;
                        hullColor = {...ERROR_COLOR, a: 0.5}
                    } else if (trajectory.volume > warningLevel) {
                        isWarning = true;
                        hullColor = {...WARNING_COLOR, a: 0.5}
                    }

                    // Keep track of specfic trajectory changes
                    addStats[trajectory.uuid] = {volume: trajectory.volume};


                    if (isWarning || isError) {
                        const uuid = generateUuid('issue');
                        issues[uuid] = {
                            uuid: uuid,
                            requiresChanges: isError,
                            title: `Robot Space Utilization`,
                            description: `Robot Space Utilization`,
                            complete: false,
                            focus: {uuid:primitive.uuid, type:'primitive'},
                            graphData: {
                                series: priorData,
                                lineColors: ["#E69F00"],
                                xAxisLabel: 'Program Iteration',
                                yAxisLabel: 'Space Usage',
                                title: ''
                            },
                            sceneData: {hulls: {spaceUsage: {vertices: spaceUtilization, color: hullColor}}}
                        }
                    }
                }
            });
        }
    });

    

    return [issues, addStats];
}