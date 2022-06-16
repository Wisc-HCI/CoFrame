import { DATA_TYPES } from "simple-vp";
import frameStyles from "../../frameStyles";
import { STATUS, STEP_TYPE } from "../Constants";
import { generateUuid } from "../generateUuid"
import { anyReachable, distance, getIDsAndStepsFromCompiled, verticesToVolume } from "../helpers";
import { Vector3} from "three";
import lodash from 'lodash';

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

export const findReachabilityIssues = ({programData}) => { // requires joint_processor to produce joints for each waypoint/location
    let issues = {};
    let usedPoses = [];

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
export const findJointSpeedIssues = ({program, programData, settings}) => {
    let issues = {};

    const warningLevel = settings["jointSpeedWarn"].value * settings['jointMaxSpeed'].value;
    const errorLevel = settings["jointSpeedErr"].value * settings['jointMaxSpeed'].value;

    let res = getIDsAndStepsFromCompiled(program, programData, STEP_TYPE.SCENE_UPDATE, "moveTrajectoryType");
    let moveTrajectoryIDs = res[0];
    let sceneUpdates = res[1];

    let timeData = {};
    let allJointData = {};
    let positionData = {};
    sceneUpdates.forEach(step => {
        if (step.source && moveTrajectoryIDs.includes(step.source)) {
            if (!(step.source in timeData)) {
                timeData[step.source] = [];
                allJointData[step.source] = {};
                positionData[step.source] = {};
            }
            timeData[step.source].push(step.time);
            Object.keys(step.data.joints).forEach(key => {
                if (!(key in allJointData[step.source])) {
                    allJointData[step.source][key] = [];
                    positionData[step.source][key] = [];
                }
                allJointData[step.source][key].push(step.data.joints[key]);
                positionData[step.source][key].push(step.data.links[jointLinkMap[key]]);
            });
        }
    });

    moveTrajectoryIDs.forEach(moveID => {
        let jointDataLength = allJointData[moveID][jointNames[0]] ? allJointData[moveID][jointNames[0]].length : 0;

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
                let calcVel = Math.abs((allJointData[moveID][jointNames[i]][j] - allJointData[moveID][jointNames[i]][j-1]) / ((timeData[moveID][j] - timeData[moveID][j-1]) / 1000));
                let curFrame = positionData[moveID][jointNames[i]][j].position;

                if (calcVel >= errorLevel) {
                    if (!hasErrorVelocity) {
                        hasErrorVelocity = true;
                        hasWarningVelocity = true;
                        shouldGraphJoint[i] = true;
                    }
                    sceneData[jointNames[i]].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: ERROR_COLOR});
                } else if (calcVel >= warningLevel) {
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
            let graphDataPoint = {x: timeData[moveID][i]};
            for (let j = 0; j < jointNames.length; j++) {
                if (shouldGraphJoint[j]) {
                    graphDataPoint[jointNameMap[jointNames[j]]] = jointVelocities[jointNames[j]][i];
                }
            }
            jointGraphData.push(graphDataPoint);
        }

        // Adding ending 0 velocity
        let graphDataPoint = {x: timeData[moveID][timeData[moveID].length-1]+1};
        for (let j = 0; j < jointNames.length; j++) {
            if (shouldGraphJoint[j]) {
                graphDataPoint[jointNameMap[jointNames[j]]] = 0;
            }
        }
        jointGraphData.push(graphDataPoint);

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
                focus: [moveID],
                graphData: {
                    series: jointGraphData,
                    xAxisLabel: 'Timestamp',
                    yAxisLabel: 'Velocity',
                    thresholds: [
                        {range: ["MIN", warningLevel], color: 'grey', label: 'OK'},
                        {range: [warningLevel, errorLevel], color: frameStyles.colors["performance"], label: 'Warning'},
                        {range: [errorLevel, "MAX"], color: frameStyles.errorColors["performance"], label: 'Error'},
                    ],
                    units: 'm/s',
                    decimal: 5,
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
export const findEndEffectorSpeedIssues = ({program, programData, settings}) => {
    let issues = {};

    const warningLevel = settings['eeSpeedWarn'].value;
    const errorLevel = settings['eeSpeedErr'].value;

    let res = getIDsAndStepsFromCompiled(program, programData, STEP_TYPE.SCENE_UPDATE, "moveTrajectoryType");
    let moveTrajectoryIDs = res[0];
    let sceneUpdates = res[1];

    let timeData = {};
    let frames = {};
    sceneUpdates.forEach(step => {
        if (step.source && moveTrajectoryIDs.includes(step.source)) {
            if (!(step.source in timeData)) {
                timeData[step.source] = [];
                frames[step.source] = [];
            }
            timeData[step.source].push(step.time);
            frames[step.source].push(step.data.links.tool0.position)
        }
    });

    
    moveTrajectoryIDs.forEach(moveID => {
        let endEffectorVelocities = [];
        let endEffectorGraphData = [];

        let hasErrorVelocity = false;
        let hasWarningVelocity = false;

        for (let i = 1; i < frames[moveID].length; i++) {
            // Adjust the time from milliseconds to seconds to calculate the velocity correctly
            let curFrame = frames[moveID][i]
            let prevFrame = frames[moveID][i - 1]
            let calcVel = distance(curFrame, prevFrame) / ((timeData[moveID][i] - timeData[moveID][i-1]) / 1000);


            if (calcVel >= errorLevel) {
                if (!hasErrorVelocity) {
                    hasErrorVelocity = true;
                }
                if (!hasWarningVelocity) {
                    hasWarningVelocity = true;
                }
                endEffectorVelocities.push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: ERROR_COLOR});
            } else if (calcVel >= warningLevel) {
                if (!hasWarningVelocity) {
                    hasWarningVelocity = true;
                }
                endEffectorVelocities.push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: WARNING_COLOR});
            } else {
                endEffectorVelocities.push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: NO_ERROR_COLOR});
            }

            endEffectorGraphData.push({x: timeData[moveID][i], 'End Effector Velocity': calcVel});
            
        }

        if (hasErrorVelocity) {
            const uuid = generateUuid('issue');
            issues[uuid] = {
                uuid: uuid,
                requiresChanges: hasErrorVelocity,
                title: `End effector moves too fast`,
                description: `The end effector moves too fast for Trajectory "${programData[moveID].name}"`,
                complete: false,
                focus: [moveID],
                graphData: {
                    series: endEffectorGraphData,
                    xAxisLabel: 'Timestamp',
                    yAxisLabel: 'Velocity',
                    thresholds: [
                        {range: ["MIN", warningLevel], color: 'grey', label: 'OK'},
                        {range: [warningLevel, errorLevel], color: frameStyles.colors["performance"], label: 'Warning'},
                        {range: [errorLevel, "MAX"], color: frameStyles.errorColors["performance"], label: 'Error'},
                    ],
                    units: 'm/s',
                    decimal: 5,
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
export const findSpaceUsageIssues = ({program, programData, stats, settings}) => {
    let issues = {};
    let addStats = {};

    const warningLevel = settings['spaceUsageWarn'].value;
    const errorLevel = settings['spaceUsageErr'].value;

    let robotWorkZone = lodash.filter(programData, function (v) { 
            return v.type === 'zoneType' && programData[v.properties.agent].type === "robotAgentType"
        })[0];
    let robotWorkZoneVolume = robotWorkZone.properties.scale.x * robotWorkZone.properties.scale.y * robotWorkZone.properties.scale.z;

    let res = getIDsAndStepsFromCompiled(program, programData, STEP_TYPE.SCENE_UPDATE, "moveTrajectoryType");
    let moveTrajectoryIDs = res[0];
    let sceneUpdates = res[1];

    let vertices = {};
    sceneUpdates.forEach(step => {
        if (step.source && moveTrajectoryIDs.includes(step.source)) {
            if (!(step.source in vertices)) {
                vertices[step.source] = [];
            }
            Object.keys(step.data.links).forEach(link => {
                vertices[step.source].push(new Vector3 (
                    step.data.links[link].position.x,
                    step.data.links[link].position.y,
                    step.data.links[link].position.z
                ));
            });
        }
    });

    moveTrajectoryIDs.forEach(moveID => {
        let volume = verticesToVolume(vertices[moveID]);
        let volumePercentage = (volume / robotWorkZoneVolume) * 100;
        let trajectory = programData[moveID].properties.trajectory;

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
        let newData = {x:i, spaceUsage: volumePercentage}
        priorData.push(newData);

        // determine whether issue should be generated
        let shouldGenIssue = false;
        // Adjust color of hull
        let hullColor = {...NO_ERROR_COLOR, a: 0.5};
        if (volumePercentage >= errorLevel) {
            isError = true;
            shouldGenIssue = true;
            hullColor = {...ERROR_COLOR, a: 0.5}
        } else if (volumePercentage >= warningLevel) {
            shouldGenIssue = true;
            hullColor = {...WARNING_COLOR, a: 0.5}
        }

        if (shouldGenIssue) {
            // Keep track of specfic trajectory changes
            addStats[trajectory] = {volume: volumePercentage};

            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
                requiresChanges: isError,
                title: `Robot Space Utilization`,
                description: `Robot Space Utilization`,
                complete: false,
                focus: [moveID],
                graphData: {
                    series: priorData,
                    xAxisLabel: 'Program Iteration',
                    yAxisLabel: 'Space Usage',
                    thresholds: [
                        {range: ["MIN", warningLevel], color: 'grey', label: 'OK'},
                        {range: [warningLevel, errorLevel], color: frameStyles.colors["performance"], label: 'Warning'},
                        {range: [errorLevel, "MAX"], color: frameStyles.errorColors["performance"], label: 'Error'},
                    ],
                    units: '%',
                    decimal: 5,
                    title: '',
                    isTimeseries: true
                },
                sceneData: {hulls: {spaceUsage: {vertices: vertices[moveID], color: hullColor}}}
            }
        }
    });

    return [issues, addStats];
}