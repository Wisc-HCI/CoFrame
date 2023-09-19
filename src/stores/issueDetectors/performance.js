import { DATA_TYPES } from "open-vp";
import frameStyles from "../../frameStyles";
import { ROOT_PATH, STEP_TYPE } from "../Constants";
import { generateUuid } from "../generateUuid"
import { anyReachable, getIDsAndStepsFromCompiled, verticesToVolume } from "../helpers";
import { distance, queryWorldPose, updateEnvironModel, updateEnvironModelQuaternion } from "../../helpers/geometry";
import { Vector3 } from "three";
import lodash from 'lodash';
import { hexToRgb } from "../../helpers/colors";

const unreachableUnusedDoc = `Reachability of a Location or Waypoint is driven by both the configuration of Location or Waypoint itself, but also the starting location and structure of the robot attempting to reach it. Since this item is not used in the program itself, this is only considered a warning, but all used locations or waypoints should be reachable.
Designing reachable locations and waypoints is both an art and a science, requiring a bit of trial and error. Try tweaking the current item until it registers as reachable.
`

const unreachableUsedDoc = `Reachability of a Location or Waypoint is driven by both the configuration of Location or Waypoint itself, but also the starting location and structure of the robot attempting to reach it. Since this item is used in the program itself, this is considered an error.
Designing reachable locations and waypoints is both an art and a science, requiring a bit of trial and error. Try tweaking the current item until it registers as reachable.
`

const jointSpeedDoc = `Joint speed is best minimized in the interest of being a proxy measure of wear-and-tear, as well as being a component of the safety assessment. 
To minimize joint speed, you can consider the following options:
- Reduce the speed of the robot during [Move Trajectory](moveTrajectoryType) actions. This potentially increases the cycle time of the program. 
- Try checking out the differences between IK-based and joint-based trajectories. Sometimes it is easier to minimize joint speed in joint-based trajectories, but they don't always produce as logical of motions.
`

const eeSpeedDoc = `End-effector speed is best minimized in the interest of being safe around human workers, given that unpredictable fast movements may more likely result in collisions with people or other moving environmental objects. 
To minimize end effector speed, you can consider the following options:
- Reduce the speed of the robot during [Move Trajectory](moveTrajectoryType) actions. This potentially increases the cycle time of the program. 
- Try checking out the differences between IK-based and joint-based trajectories. Sometimes it is easier to minimize end effector speed in IK-based trajectories, but they are sometimes harder to get valid solutions for.
`

const payloadDoc = `Payload is determined by the model of the robot being used, and each [Thing](thingType) or [Tool](toolType) that the robot picks up must be below that payload weight. 
If some object is above the payload weight, the robot will be unable to pick it up.

In the event of a payload error, an alternative program structure should be considered that doesn't involve moving the specified object.
`

const spaceUsageDoc = `Space usage is a measure of how much space the robot occupies while performing a [robot motion](moveTrajectoryType). The visual associated with this issue shows the 3D volume of the space occupied in this motion.
To reduce space usage, consider doing the following:

> [primary]Where possible, move the robot in a way that puts the robot's end-effector closest to its base for the majority of the trajectory. Note, you will have to balance this type of movement with the possibility of creating pinch points.
`

export const findReachabilityIssues = ({program, programData, compiledData}) => { // requires joint_processor to produce joints for each waypoint/location
    let issues = {};
    let checkedPoses = [];
    let usedPoses = [];

    // Check to see if a location or waypoint is used in the compiled data
    compiledData[program.id]?.[ROOT_PATH]?.steps.forEach(step=>{
        if (step.type === STEP_TYPE.ACTION_START && programData[step.source].type === 'moveTrajectoryType') {
            const moveTrajectory = programData[step.source];
            const trajectory = programData[moveTrajectory.properties.trajectory];
            const poses = [
                trajectory.properties.startLocation,
                ...trajectory.properties.waypoints,
                trajectory.properties.endLocation,
            ]
            poses.forEach(pose=>{
                if (!usedPoses.includes(pose)) {
                    usedPoses.push(pose)
                }
            })
        }
    }),

    // Enumerate locations and add warnings for those not used in the program.
    Object.values(programData).filter(v => v.type === 'locationType' && v.dataType === DATA_TYPES.INSTANCE).forEach(location=>{
        if (!anyReachable(location) && checkedPoses.includes(location.id)) {
            checkedPoses.push(location.id)
            const used = usedPoses.includes(location.id)
            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
                requiresChanges: used,
                title: used ? `Location "${location.name}" not reachable` : `Unused Location "${location.name}" not reachable`,
                description: used ? `Location "${location.name}" is not reachable by the robot, and is used in the program.` : `Location "${location.name}" is not reachable by the robot, but isn't used in the program.`,
                featuredDocs: {[location.id]: used ? unreachableUsedDoc : unreachableUnusedDoc},
                complete: false,
                focus: [location.id],
                graphData: null
            }
        }
    });

    // Enumerate waypoints and add warnings for those not used in the program.
    Object.values(programData).filter(v => v.type === 'waypointType' && v.dataType === DATA_TYPES.INSTANCE).forEach(waypoint=>{
        if (!anyReachable(waypoint) && checkedPoses.includes(waypoint.id)) {
            checkedPoses.push(waypoint.id)
            const used = usedPoses.includes(location.id)
            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
                requiresChanges: used,
                title: used ? `Waypoint "${waypoint.name}" not reachable` : `Unused Waypoint "${waypoint.name}" not reachable`,
                description: used ? `Waypoint "${waypoint.name}" is not reachable by the robot, and is used in the program.` : `Waypoint "${waypoint.name}" is not reachable by the robot, but isn't used in the program.`,
                featuredDocs: {[location.id]:used ? unreachableUsedDoc : unreachableUnusedDoc},
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
        if (programData[step.source].type === 'moveTrajectoryType') {
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

        timeData[source] = [0];

        let sceneData = {};
        let jointVelocities = {};
        let jointGraphData = [];

        jointNames.forEach(joint => {
            jointVelocities[joint] = [0];
            sceneData[joint] = [];
        });

        // Base update
        let tindex = 0;
        let found = false;
        while (!found) {
            if (moveTrajectorySteps?.[source]?.[tindex]?.data?.links) {
                Object.keys(moveTrajectorySteps?.[source]?.[tindex]?.data?.links).forEach(link => {
                    if (typeof(moveTrajectorySteps[source][tindex].data.links[link].rotation?.w) === typeof(1)) {
                        environmentModel = updateEnvironModelQuaternion(environmentModel, link, moveTrajectorySteps[source][tindex].data.links[link].position, moveTrajectorySteps[source][tindex].data.links[link].rotation);    
                        found = true;
                    } else {
                        environmentModel = updateEnvironModel(environmentModel, link, moveTrajectorySteps[source][tindex].data.links[link].position, moveTrajectorySteps[source][tindex].data.links[link].rotation);
                        found = true;
                    }
                });
            }
            tindex += 1;
            if (tindex >= moveTrajectorySteps?.[source].length) {
                found = true;
            }
        }

        let prevMoveTrajectoryStep = null;
        let initialTime = moveTrajectorySteps[source][0].time;
        moveTrajectorySteps[source].forEach(step => {
            if (count > 0) {
                timeData[source].push(step.time - initialTime);
                // let prevousPositions = {}
                // jointNames.forEach(joint => {
                //     prevousPositions[joint] = {...queryWorldPose(environmentModel, jointLinkMap[joint], '')};
                // });
                if (step.type === STEP_TYPE.SCENE_UPDATE) {
                    Object.keys(step.data.links).forEach(link => {
                        if (typeof(step.data.links[link].rotation?.w) === typeof(1)) {
                            environmentModel = updateEnvironModelQuaternion(environmentModel, link, step.data.links[link].position, step.data.links[link].rotation);
                        } else {
                            environmentModel = updateEnvironModel(environmentModel, link, step.data.links[link].position, step.data.links[link].rotation);
                        }
                    });

                    jointNames.forEach(joint => {
                        const prevJointValue = prevMoveTrajectoryStep.data.joints[joint];
                        const prevTime = prevMoveTrajectoryStep.time;
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
                    prevMoveTrajectoryStep = {...step};
                } else {
                    Object.keys(jointVelocities).forEach(jointName => {
                        jointVelocities[jointName].push(jointVelocities[jointName][jointVelocities[jointName].length - 1]);
                    });
                }
                count += 1;
            }
            if (count === 0 && step.type === STEP_TYPE.SCENE_UPDATE) {
                count += 1;
                prevMoveTrajectoryStep = {...step};
            }
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
                featuredDocs: {[source]:jointSpeedDoc},
                complete: false,
                focus: [source],
                graphData: {
                    series: jointGraphData,
                    xAxisLabel: 'Time',
                    yAxisLabel: 'Velocity',
                    thresholds: [
                        {range: ["MIN", warningLevel], color: 'grey', label: 'OK'},
                        {range: [warningLevel, errorLevel], color: frameStyles.colors["performance"], label: 'Warning'},
                        {range: [errorLevel, "MAX"], color: frameStyles.errorColors["performance"], label: 'Error'},
                    ],
                    units: 'rad/s',
                    decimals: 2,
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
    let initialTimeData = {}
    let linkData = {};
    sceneUpdates.forEach(step => {
        if (step.source && moveTrajectoryIDs.includes(step.source)) {
            if (!(step.source in timeData)) {
                timeData[step.source] = [];
                linkData[step.source] = [];
                initialTimeData[step.source] = step.time;
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
            if (typeof(linkData[moveID][0][link].rotation?.w) === typeof(1)) {
                environmentModel = updateEnvironModelQuaternion(environmentModel, link, {...linkData[moveID][0][link].position}, {...linkData[moveID][0][link].rotation});
            } else {
                environmentModel = updateEnvironModel(environmentModel, link, {...linkData[moveID][0][link].position}, {...linkData[moveID][0][link].rotation});
            }
            
        });

        for (let i = 1; i < linkData[moveID].length; i++) {
            // Pull previous end-effector position
            let prevFrameData = queryWorldPose(environmentModel, gripperId+'-gripOffset', '');
            let prevFrame = prevFrameData.position;

            // Update model to current frame
            Object.keys(linkData[moveID][i]).forEach(link => {
                if (typeof(linkData[moveID][i][link].rotation?.w) === typeof(1)) {
                    environmentModel = updateEnvironModelQuaternion(environmentModel, link, {...linkData[moveID][i][link].position}, {...linkData[moveID][i][link].rotation});
                } else {
                    environmentModel = updateEnvironModel(environmentModel, link, {...linkData[moveID][i][link].position}, {...linkData[moveID][i][link].rotation});
                }
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

            endEffectorGraphData.push({x: timeData[moveID][i] - initialTimeData[moveID], 'End Effector Velocity': calcVel});
            
        }

        if (hasErrorVelocity) {
            const uuid = generateUuid('issue');
            issues[uuid] = {
                id: uuid,
                requiresChanges: hasErrorVelocity,
                title: `End effector moves too fast`,
                description: `The end effector moves too fast for Trajectory "${programData[moveID].name}"`,
                featuredDocs: {[moveID]:eeSpeedDoc},
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
                    decimals: 5,
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
                        featuredDocs: {[step.source]:payloadDoc},
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
                        featuredDocs: {[step.source]:payloadDoc},
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
                featuredDocs: {[moveID]:spaceUsageDoc},
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
                    decimals: 5,
                    title: '',
                    isTimeseries: true
                },
                sceneData: {hulls: {spaceUsage: {vertices: vertices[moveID], color: hullColor}}}
            }
        }
    });

    return [issues, addStats];
}
