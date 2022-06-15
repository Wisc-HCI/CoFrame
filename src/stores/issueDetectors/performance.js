import { DATA_TYPES } from "simple-vp";
import frameStyles from "../../frameStyles";
import { STATUS, STEP_TYPE } from "../Constants";
import { generateUuid } from "../generateUuid"
import { anyReachable, distance, stepsToVertices, verticesToVolume } from "../helpers";

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

export const findReachabilityIssues = ({programData}) => { // requires joint_processor to produce joints for each waypoint/location
    let issues = {};
    let usedPoses = [];
    // First check through the trajectories that are used.
    Object.values(programData).filter(v => v.type === "trajectoryType" && v.dataType === DATA_TYPES.INSTANCE).forEach(trajectory=>{
        // Check through the start locations. If not defined, this is resolved in program quality.
        if (trajectory.properties.startLocation) {
            const startLocation = programData[programData[trajectory.properties.startLocation].ref];
            if (!anyReachable(startLocation) && usedPoses.indexOf(startLocation.id) < 0) {
                usedPoses.push(startLocation.id)
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    id: uuid,
                    requiresChanges: true,
                    title: `Location "${startLocation.name}" not reachable`,
                    description: `Location "${startLocation.name}" is not reachable by the robot, but is used in the program.`,
                    complete: false,
                    focus: [startLocation.id],
                    graphData: null
                }
            }
        };
        // Check through the end locations. If not defined, this is resolved in program quality.
        if (trajectory.properties.endLocation) {
            const endLocation = programData[programData[trajectory.properties.endLocation].ref];
            if (!anyReachable(endLocation) && usedPoses.indexOf(endLocation.id) < 0) {
                usedPoses.push(endLocation.id)
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    id: uuid,
                    requiresChanges: true,
                    title: `Location "${endLocation.name}" not reachable`,
                    description: `Location "${endLocation.name}" is not reachable by the robot, but is used in the program.`,
                    complete: false,
                    focus: [endLocation.id],
                    graphData: null
                }
            }
        }
        // Now go through the waypoints of the trajectory
        trajectory.properties.waypoints.forEach(waypoint_uuid=>{
            const waypoint = programData[programData[waypoint_uuid].ref];
            if (!anyReachable(waypoint)  && usedPoses.indexOf(waypoint.id) < 0) {
                usedPoses.push(waypoint.id)
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    id: uuid,
                    requiresChanges: true,
                    title: `Waypoint "${waypoint.name}" not reachable`,
                    description: `Waypoint "${waypoint.name}" is not reachable by the robot, but is used in the program.`,
                    complete: false,
                    focus: [waypoint.id],
                    graphData: null
                }
            }
        });
    });

    // The initialize skill takes in a location parameter, so check for calls that use it.
    Object.values(programData).filter(v => v.dataType === DATA_TYPES.CALL).forEach(primitive=>{
        Object.keys(primitive.properties).filter(key=>key.includes('location')).forEach(key=>{
            const location_uuid = primitive.properties[key];
            let location = location_uuid ? programData[programData[location_uuid].ref] : null;
            if (location && location.properties.reachability && !anyReachable(location)) {
                const uuid = generateUuid('issue');
                issues[uuid] = {
                    id: uuid,
                    requiresChanges: true,
                    title: `Initial robot Location "${location.name}" not reachable`,
                    description: `Location "${location.name}" is used for initialization but is not reachable.`,
                    complete: false,
                    focus: [location.id],
                    graphData: null
                }
            }
        });
    });

    // Enumerate locations and add warnings for those not used in the program.
    Object.values(programData).filter(v => v.type === 'locationType' && v.dataType === DATA_TYPES.INSTANCE).forEach(location=>{
        if (!anyReachable(location) && usedPoses.indexOf(location.id) < 0) {
            usedPoses.push(location.id)
            const uuid = generateUuid('issue');
            issues[uuid] = {
                uuid: uuid,
                requiresChanges: false,
                title: `Unused Location "${location.name}" not reachable`,
                description: `Location "${location.name}" is not reachable by the robot, but isn't used in the program.`,
                complete: false,
                focus: [location.id],
                graphData: null
            }
        }
    });

    // Enumerate waypoints and add warnings for those not used in the program.
    Object.values(programData).filter(v => v.type === 'waypointType' && v.dataType === DATA_TYPES.INSTANCE).forEach(waypoint=>{
        if (!anyReachable(waypoint) && usedPoses.indexOf(waypoint.id) < 0) {
            usedPoses.push(waypoint.id)
            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
                requiresChanges: false,
                title: `Unused Waypoint "${waypoint.name}" not reachable`,
                description: `Waypoint "${waypoint.name}" is not reachable by the robot, but isn't used in the program.`,
                complete: false,
                focus: [waypoint.id],
                graphData: null
            }
        }
    })
    return [issues, {}];
}

// requires trace processor, joint speed grader (can also use intermediate type)
export const findJointSpeedIssues = ({programData, settings}) => {
    let issues = {};

    const warningLevel = settings["jointSpeedWarn"].value * settings['jointMaxSpeed'].value;
    const errorLevel = settings["jointSpeedErr"].value * settings['jointMaxSpeed'].value;

    Object.values(programData).filter(v => v.type === 'moveTrajectoryType').forEach(moveTrajectory => {
        // Filter to just steps that affect the joint motions
        let steps = moveTrajectory.properties.compiled["{}"] ? moveTrajectory.properties.compiled["{}"].steps.filter(v => v.type === STEP_TYPE.SCENE_UPDATE) : [];
        
        // Build the arrays for time, joint values (according to time), and link positions (according to time)
        let timeData = [];
        let allJointData = {};
        let positionData = {};
        for (let i = 0; i < steps.length; i++) {
            timeData.push(steps[i].time);
            Object.keys(steps[i].data.joints).forEach(key => {
                if (!(key in allJointData)) {
                    allJointData[key] = [];
                    positionData[key] = [];
                }
                allJointData[key].push(steps[i].data.joints[key]);
                positionData[key].push(steps[i].data.links[jointLinkMap[key]]);
            })
        }
        let jointDataLength = allJointData[jointNames[0]] ? allJointData[jointNames[0]].length : 0;

        let sceneData = {};
        let jointVelocities = {};
        let jointGraphData = [];

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
                // Adjust the time from milliseconds to seconds to calculate the velocity correctly
                let calcVel = Math.abs((allJointData[jointNames[i]][j] - allJointData[jointNames[i]][j-1]) / ((timeData[j] - timeData[j-1]) / 1000));
                let curFrame = positionData[jointNames[i]][j].position;

                if (calcVel > errorLevel) {
                    if (!hasErrorVelocity) {
                        hasErrorVelocity = true;
                        hasWarningVelocity = true;
                        shouldGraphJoint[i] = true;
                    }
                    sceneData[jointNames[i]].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: ERROR_COLOR});
                } else if (calcVel > warningLevel) {
                    if (!hasWarningVelocity) {
                        hasWarningVelocity = true;
                        shouldGraphJoint[i] = true;
                    }
                    sceneData[jointNames[i]].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: WARNING_COLOR});
                } else {
                    sceneData[jointNames[i]].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: NO_ERROR_COLOR});
                }
                jointVelocities[jointNames[i]].push(calcVel);
            }
        }

        // Filter and format graph data
        for (let i = 0; i < jointDataLength; i++) {
            let graphDataPoint = {x: timeData[i]};
            for (let j = 0; j < jointNames.length; j++) {
                if (shouldGraphJoint[j]) {
                    graphDataPoint[jointNameMap[jointNames[j]]] = jointVelocities[jointNames[j]][i];
                }
            }
            jointGraphData.push(graphDataPoint);
        }

        // Adding ending 0 velocity
        for (let j = 0; j < jointNames.length; j++) {
            let graphDataPoint = {x: timeData[timeData.length-1]+1};
            if (shouldGraphJoint[j]) {
                graphDataPoint[jointNameMap[jointNames[j]]] = 0;
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
        if (hasWarningVelocity || hasErrorVelocity) {
            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
                requiresChanges: hasErrorVelocity,
                title: `Robot joint(s) move too fast`,
                description: `The robot's joint speeds are too fast`,
                complete: false,
                focus: [moveTrajectory.id],
                graphData: {
                    series: jointGraphData,
                    xAxisLabel: 'Timestamp',
                    yAxisLabel: 'Velocity',
                    warningThreshold: warningLevel,
                    errorThreshold: errorLevel,
                    warningColor: frameStyles.colors["performance"],
                    errorColor: frameStyles.errorColors["performance"],
                    title: '',
                    isTimeseries: true
                },
                sceneData: {vertices: sceneData}
            }
        }
    });

    return [issues, {}];
}

// requires trace processor, end effector grader + intermediate end effector speed interediate type
export const findEndEffectorSpeedIssues = ({programData, settings}) => {
    let issues = {};

    const warningLevel = settings['eeSpeedWarn'].value;
    const errorLevel = settings['eeSpeedErr'].value;
    
    Object.values(programData).filter(v => v.type === "moveTrajectoryType").forEach(primitive=>{
        // Filter to just steps that affect the joint motions
        let steps = primitive.properties.compiled["{}"] ? primitive.properties.compiled["{}"].steps.filter(v => v.type === STEP_TYPE.SCENE_UPDATE) : [];
        
        // Build the arrays for time, joint values (according to time), and link positions (according to time)
        let timeData = [];
        let frames = [];
        for (let i = 0; i < steps.length; i++) {
            timeData.push(steps[i].time);
            frames.push(steps[i].data.links.tool0.position)
        }

        let endEffectorVelocities = [];
        let endEffectorGraphData = [];

        let hasErrorVelocity = false;
        let hasWarningVelocity = false;

        for (let i = 1; i < frames.length; i++) {
            // Adjust the time from milliseconds to seconds to calculate the velocity correctly
            let calcVel = distance(frames[i], frames[i-1]) / ((timeData[i] - timeData[i-1]) / 1000);


            if (calcVel > errorLevel) {
                if (!hasErrorVelocity) {
                    hasErrorVelocity = true;
                }
                if (!hasWarningVelocity) {
                    hasWarningVelocity = true;
                }
                endEffectorVelocities.push({position: {x: frames[i].x, y: frames[i].y, z: frames[i].z}, color: ERROR_COLOR});
            } else if (calcVel > warningLevel) {
                if (!hasWarningVelocity) {
                    hasWarningVelocity = true;
                }
                endEffectorVelocities.push({position: {x: frames[i].x, y: frames[i].y, z: frames[i].z}, color: WARNING_COLOR});
            } else {
                endEffectorVelocities.push({position: {x: frames[i].x, y: frames[i].y, z: frames[i].z}, color: NO_ERROR_COLOR});
            }

            endEffectorGraphData.push({x: timeData[i], 'End Effector Velocity': calcVel});
            
        }

        if (hasErrorVelocity) {
            const uuid = generateUuid('issue');
            issues[uuid] = {
                uuid: uuid,
                requiresChanges: hasErrorVelocity,
                title: `End effector moves too fast`,
                description: `The end effector moves too fast for Trajectory "${primitive.name}"`,
                complete: false,
                focus: [primitive.id],
                graphData: {
                    series: endEffectorGraphData,
                    xAxisLabel: 'Timestamp',
                    yAxisLabel: 'Velocity',
                    warningThreshold: warningLevel,
                    errorThreshold: errorLevel,
                    warningColor: frameStyles.colors["performance"],
                    errorColor: frameStyles.errorColors["performance"],
                    title: '',
                    isTimeseries: true
                },
                sceneData: {vertices: {endEffector: endEffectorVelocities}}
            }
        }
    });

    return [issues, {}];
}

export const findPayloadIssues = (_) => { // Shouldn't change during a trajectory so more of a check on thing weight vs. robot payload (e.g., 3kg in 1g)
    let issues = {};

    return [issues, {}];
}

// Requires a convex hall operation on joint frames in traces. This volume can be compared against whole workcell (fraction) and can be used for intersection with extruded human occupancy zones 
export const findSpaceUsageIssues = ({programData, stats, settings}) => {
    let issues = {};
    let addStats = {};

    const warningLevel = settings['spaceUsageWarn'].value;
    const errorLevel = settings['spaceUsageErr'].value;
    
    Object.values(programData).filter(v => v.type === "moveTrajectoryType").forEach(primitive=>{
        let steps = primitive.properties.compiled["{}"] ? primitive.properties.compiled["{}"].steps.filter(v => v.type === STEP_TYPE.SCENE_UPDATE) : [];
        let verticies = stepsToVertices(steps);
        let volume = verticesToVolume(verticies);
        let trajectory = primitive.properties.trajectory;
        
        let isError = false;

        // get prior values
        let priorData = [];
        let i = 0;
        for (i = 0; i < stats.length; i++) {
            if (stats[i][trajectory] && stats[i][trajectory].volume) {
                priorData.push({x: i, spaceUsage: stats[i][trajectory].volume});
            }
        }
        // add new one
        let newData = {x:i, spaceUsage: volume}
        priorData.push(newData);

        let shouldGenIssue = false;

        // Adjust color of hull
        let hullColor = {...NO_ERROR_COLOR, a: 0.5};
        if (volume > errorLevel) {
            isError = true;
            shouldGenIssue = true;
            hullColor = {...ERROR_COLOR, a: 0.5}
        } else if (volume > warningLevel) {
            shouldGenIssue = true;
            hullColor = {...WARNING_COLOR, a: 0.5}
        }

        if (shouldGenIssue) {
            // Keep track of specfic trajectory changes
            addStats[trajectory] = {volume: volume};

            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
                requiresChanges: isError,
                title: `Robot Space Utilization`,
                description: `Robot Space Utilization`,
                complete: false,
                focus: [primitive.id],
                graphData: {
                    series: priorData,
                    xAxisLabel: 'Program Iteration',
                    yAxisLabel: 'Space Usage',
                    warningThreshold: warningLevel,
                    errorThreshold: errorLevel,
                    warningColor: frameStyles.colors["performance"],
                    errorColor: frameStyles.errorColors["performance"],
                    title: '',
                    isTimeseries: true
                },
                sceneData: {hulls: {spaceUsage: {vertices: verticies, color: hullColor}}}
            }
        }
    });

    return [issues, addStats];
}