import { DATA_TYPES } from "simple-vp";
import frameStyles from "../../frameStyles";
import { ROOT_PATH, STEP_TYPE } from "../Constants";
import { generateUuid } from "../generateUuid"
import { anyReachable, getIDsAndStepsFromCompiled, verticesToVolume } from "../helpers";
import { distance, queryWorldPose, updateEnvironModel } from "../../helpers/geometry";
import { Vector3 } from "three";
import lodash from 'lodash';
import { hexToRgb } from "../../helpers/colors";

export const findReachabilityIssues = ({programData}) => { // requires joint_processor to produce joints for each waypoint/location
    let issues = {};
    let usedPoses = [];

    // Enumerate locations and add warnings for those not used in the program.
    Object.values(programData).filter(v => v.type === 'locationType' && v.dataType === DATA_TYPES.INSTANCE).forEach(location=>{
        if (!anyReachable(location) && usedPoses.indexOf(location.id) < 0) {
            usedPoses.push(location.id)
            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
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
export const findJointSpeedIssues = ({program, programData, settings, environmentModel, compiledData}) => {
    let issues = {};

    const warningLevel = settings["jointSpeedWarn"].value * settings['jointMaxSpeed'].value;
    const errorLevel = settings["jointSpeedErr"].value * settings['jointMaxSpeed'].value;

    let timeData = {};
    const robotAgent = lodash.filter(programData, function (v) { return v.type === 'robotAgentType' })[0];
    const jointLinkMap = robotAgent.properties.jointLinkMap;
    const jointNames = Object.keys(jointLinkMap);
    let jointNameMap = {}
    jointNames.forEach(name => {
        jointNameMap[name] = name.replace(/\w\S*/g, (w) => (w.replace(/^\w/, (c) => c.toUpperCase())));
    });
    
    let errorWarning = {};
    jointNames.forEach(jName => {
        errorWarning[jName] = {error: false, warning: false};
    });

    let moveTrajectorySteps = {};
    let moveIds = [];
    compiledData[program.id]?.[ROOT_PATH]?.steps?.forEach(step => {
        if (step.type === STEP_TYPE.SCENE_UPDATE && programData[step.source].type === 'moveTrajectoryType') {
            if (!(moveIds.includes(step.source))) {
                moveIds.push(step.source);
                moveTrajectorySteps[step.source] = [];
            }
            moveTrajectorySteps[step.source].push({...step});
        }
    });

    let moveTracjectoryIssues = [];

    moveIds.forEach(source => {
        let count = 0;

        timeData[source] = [];

        let sceneData = {};
        let jointVelocities = {};
        let jointGraphData = [];

        jointNames.forEach(joint => {
            jointVelocities[joint] = [0];
            sceneData[joint] = [];
        });

        // Base update
        Object.keys(moveTrajectorySteps[source][0].data.links).forEach(link => {
            environmentModel = updateEnvironModel(environmentModel, link, moveTrajectorySteps[source][0].data.links[link].position, moveTrajectorySteps[source][0].data.links[link].rotation);
        });

        moveTrajectorySteps[source].forEach(step => {
            if (count > 0) {
                timeData[source].push(step.time);
                let prevousPositions = {}
                jointNames.forEach(joint => {
                    prevousPositions[joint] = {...queryWorldPose(environmentModel, jointLinkMap[joint], '')};
                });

                Object.keys(step.data.links).forEach(link => {
                    environmentModel = updateEnvironModel(environmentModel, link, step.data.links[link].position, step.data.links[link].rotation);
                });

                jointNames.forEach(joint => {
                    const prevJointValue = moveTrajectorySteps[source][count-1].data.joints[joint];
                    const prevTime = moveTrajectorySteps[source][count-1].time;
                    const curJointValue = step.data.joints[joint];
                    const curTime = step.time;
                    const currentPosition = queryWorldPose(environmentModel, jointLinkMap[joint], '');
                    const curFrame = currentPosition.position;
                    const calcVel = Math.abs((curJointValue - prevJointValue) / ((curTime - prevTime) / 1000));

                    if (calcVel >= errorLevel) {
                        errorWarning[joint].error = true;
                        errorWarning[joint].warning = true;
                        sceneData[joint].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: hexToRgb(frameStyles.errorColors["performance"])});
                    } else if (calcVel >= warningLevel) {
                        errorWarning[joint].warning = true;
                        sceneData[joint].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: hexToRgb(frameStyles.colors["performance"])});
                    } else {
                        sceneData[joint].push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: hexToRgb(frameStyles.colors["default"])});
                    }
                    jointVelocities[joint].push(calcVel);
                });
            }
            count += 1;
        });

        // Filter and format graph data
        for (let i = 0; i < timeData[source].length; i++) {
            let graphDataPoint = {x: timeData[source][i]};
            jointNames.forEach(joint => {
                if (errorWarning[joint].error || errorWarning[joint].warning) {
                    graphDataPoint[jointNameMap[joint]] = jointVelocities[joint][i];
                }
            })
            jointGraphData.push(graphDataPoint);
        }

        // Adding ending 0 velocity
        let graphDataPoint = {x: timeData[source][timeData[source].length-1]+1};
        jointNames.forEach(joint => {
            if (errorWarning[joint].error || errorWarning[joint].warning) {
                graphDataPoint[jointNameMap[joint]] = 0;
            }
        })
        jointGraphData.push(graphDataPoint);

        let err = false;
        let warn = false;
        jointNames.forEach(joint => {
            err = err || errorWarning[joint].error;
            warn = warn || errorWarning[joint].warning;
        });

        // Build issue
        if (!moveTracjectoryIssues.includes(source) && (err || warn)) {
            moveTracjectoryIssues.push(source);
            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
                requiresChanges: err,
                title: `Robot joint(s) move too fast`,
                description: `The robot's joint speeds are too fast`,
                complete: false,
                focus: [source],
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
export const findEndEffectorSpeedIssues = ({program, programData, settings, environmentModel, compiledData}) => {
    let issues = {};

    const warningLevel = settings['eeSpeedWarn'].value;
    const errorLevel = settings['eeSpeedErr'].value;

    let res = getIDsAndStepsFromCompiled(program, programData, STEP_TYPE.SCENE_UPDATE, "moveTrajectoryType", compiledData);
    let gripperId = Object.values(programData).filter(d=>d.type==='gripperType'&&d.dataType===DATA_TYPES.INSTANCE)[0].id;
    let moveTrajectoryIDs = res[0];
    let sceneUpdates = res[1];

    let timeData = {};
    let linkData = {};
    sceneUpdates.forEach(step => {
        if (step.source && moveTrajectoryIDs.includes(step.source)) {
            if (!(step.source in timeData)) {
                timeData[step.source] = [];
                linkData[step.source] = [];
            }
            timeData[step.source].push(step.time);
            linkData[step.source].push({...step.data.links});
        }
    });

    
    moveTrajectoryIDs.forEach(moveID => {
        let endEffectorVelocities = [];
        let endEffectorGraphData = [];

        let hasErrorVelocity = false;
        let hasWarningVelocity = false;

        // Initially update the model
        Object.keys(linkData[moveID][0]).forEach(link => {
            environmentModel = updateEnvironModel(environmentModel, link, {...linkData[moveID][0][link].position}, {...linkData[moveID][0][link].rotation});
        });

        for (let i = 1; i < linkData[moveID].length; i++) {
            // Pull previous end-effector position
            let prevFrameData = queryWorldPose(environmentModel, gripperId+'-gripOffset', '');
            let prevFrame = prevFrameData.position;

            // Update model to current frame
            Object.keys(linkData[moveID][i]).forEach(link => {
                environmentModel = updateEnvironModel(environmentModel, link, {...linkData[moveID][i][link].position}, {...linkData[moveID][i][link].rotation});
            });

            // Pull current end-effector position
            let curFrameData = queryWorldPose(environmentModel, gripperId+'-gripOffset', '');
            let curFrame = curFrameData.position;

            // Adjust the time from milliseconds to seconds to calculate the velocity correctly
            let calcVel = distance(curFrame, prevFrame) / ((timeData[moveID][i] - timeData[moveID][i-1]) / 1000);


            if (calcVel >= errorLevel) {
                if (!hasErrorVelocity) {
                    hasErrorVelocity = true;
                }
                if (!hasWarningVelocity) {
                    hasWarningVelocity = true;
                }
                endEffectorVelocities.push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: hexToRgb(frameStyles.errorColors["performance"])});
            } else if (calcVel >= warningLevel) {
                if (!hasWarningVelocity) {
                    hasWarningVelocity = true;
                }
                endEffectorVelocities.push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: hexToRgb(frameStyles.colors["performance"])});
            } else {
                endEffectorVelocities.push({position: {x: curFrame.x, y: curFrame.y, z: curFrame.z}, color: hexToRgb(frameStyles.colors["default"])});
            }

            endEffectorGraphData.push({x: timeData[moveID][i], 'End Effector Velocity': calcVel});
            
        }

        if (hasErrorVelocity) {
            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
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

export const findPayloadIssues = ({program, programData, settings, compiledData}) => { // Shouldn't change during a trajectory so more of a check on thing weight vs. robot payload (e.g., 3kg in 1g)
    let issues = {};

    let warningLevel = settings["payloadWarn"].value;
    let errorLevel = settings["payloadErr"].value;

    let tracked = [];

    compiledData[program.id]?.[ROOT_PATH]?.steps?.forEach(step => {
        let source = programData[step.source];

        if (step.type === STEP_TYPE.SCENE_UPDATE && source && source.type === "moveGripperType") {
            let thingId = step.data.thing.id ? step.data.thing.id : step.data.thing;

            if (thingId) {
                let thing = programData[thingId];

                if (thing && thing.properties.weight >= errorLevel && !tracked.includes(step.source)) {
                    tracked.push(step.source);
                    let id = generateUuid('issue');
                    issues[id] = {
                        id: id,
                        requiresChanges: true,
                        title: `Payload threshold exceeded`,
                        description: `The robot is attempting to grab a thing that exceeds the set payload threshold`,
                        complete: false,
                        focus: [step.source],
                        graphData: null,
                        sceneData: null
                    }
                } else if (thing && thing.properties.weight >= warningLevel && !tracked.includes(step.source)) {
                    tracked.push(step.source);

                    tracked.push(step.source);
                    let id = generateUuid('issue');
                    issues[id] = {
                        id: id,
                        requiresChanges: false,
                        title: `Approaching payload threshold`,
                        description: `The robot is attempting to grab a thing that is close to the payload threshold`,
                        complete: false,
                        focus: [step.source],
                        graphData: null,
                        sceneData: null
                    }
                }
            }
        }
    });

    return [issues, {}];
}

// Requires a convex hall operation on joint frames in traces. This volume can be compared against whole workcell (fraction) and can be used for intersection with extruded human occupancy zones 
export const findSpaceUsageIssues = ({program, programData, stats, settings, compiledData}) => {
    let issues = {};
    let addStats = {};

    const warningLevel = settings['spaceUsageWarn'].value;
    const errorLevel = settings['spaceUsageErr'].value;

    let robotWorkZone = lodash.filter(programData, function (v) { 
            return v.type === 'zoneType' && programData[v.properties.agent].type === "robotAgentType"
        })[0];
    let robotWorkZoneVolume = robotWorkZone.properties.scale.x * robotWorkZone.properties.scale.y * robotWorkZone.properties.scale.z;

    let res = getIDsAndStepsFromCompiled(program, programData, STEP_TYPE.SCENE_UPDATE, "moveTrajectoryType", compiledData);
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
        let hullColor = {...hexToRgb(frameStyles.colors['default']), a: 0.5};
        if (volumePercentage >= errorLevel) {
            isError = true;
            shouldGenIssue = true;
            hullColor = {...hexToRgb(frameStyles.errorColors['performance']), a: 0.5}
        } else if (volumePercentage >= warningLevel) {
            shouldGenIssue = true;
            hullColor = {...hexToRgb(frameStyles.colors['performance']), a: 0.5}
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
