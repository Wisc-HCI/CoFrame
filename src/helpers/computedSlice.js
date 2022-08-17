import lodash from "lodash";
import {
    MAX_GRIPPER_DISTANCE_DIFF,
    MAX_GRIPPER_ROTATION_DIFF,
    STEP_TYPE,
} from "../stores/Constants";
import { generateUuid } from "../stores/generateUuid";
import {
    interpolateScalar,
    checkHandThresholds
} from '../stores/helpers';
import { map } from 'lodash';
import { likProximityAdjustment } from "./conversion";
import {
    addGraspPointToModel,
    addToEnvironModel,
    createEnvironmentModel,
    distance,
    getAllChildrenFromModel,
    getUserDataFromModel,
    quaternionFromEuler,
    queryWorldPose,
    updateEnvironModel
} from "./geometry";
import { Quaternion } from "three";

const pinchColorFromMagnitude = (magnitude = 0) => {
    return {
        r: 204 + 29 * magnitude,
        g: 121 - 68 * magnitude,
        b: 167 - 15 * magnitude,
        a: 0.3,
    };
};

const centerPoint = (point1, point2) => {
    return [
        (point1[0] + point2[0]) / 2,
        (point1[1] + point2[1]) / 2,
        (point1[2] + point2[2]) / 2,
    ];
};

const pinchPointVisualsByStep = (pairedLinks, proximity, previousDistances) => {
    let pinchPoints = {};

    if (!proximity) {
        return pinchPoints;
    }

    pairedLinks.forEach((pair) => {
        let link1 = pair["link1"];
        let link2 = pair["link2"];

        if (!pinchPoints[link1]) {
            pinchPoints[link1] = {};
        }

        if (
            proximity[link1] &&
            proximity[link1][link2] &&
            checkHandThresholds(proximity[link1][link2].distance) &&
            previousDistances[link1] &&
            previousDistances[link1][link2] &&
            previousDistances[link1][link2].distance -
            proximity[link1][link2].distance > 0
        ) {
            let errorMagnitude =
                1 / Math.pow(Math.E, proximity[link1][link2].distance);
            let errorPosition =
                proximity[link1][link2]?.points?.length > 0
                    ? centerPoint(
                        proximity[link1][link2].points[0],
                        proximity[link1][link2].points[1]
                    )
                    : [0, 0, 0];

            pinchPoints[link1][link2] = {
                scale: {
                    x: errorMagnitude * 0.1,
                    y: errorMagnitude * 0.1,
                    z: errorMagnitude * 0.1,
                },
                color: pinchColorFromMagnitude(errorMagnitude),
                position: {
                    x: errorPosition[0],
                    y: errorPosition[1],
                    z: errorPosition[2],
                },
            };
        } else {
            pinchPoints[link1][link2] = {
                scale: { x: 0, y: 0, z: 0 },
                color: { r: 0, g: 0, b: 0, a: 0 },
                position: { x: 0, y: 0, z: 0 },
            };
        }
    });
    return pinchPoints;
};

const stepsToAnimatedPinchPoints = (steps) => {
    if (steps.length === 0) {
        return {};
    }

    let tempAnimatedPinchPoints = {};
    let structure = {};

    Object.keys(steps[0].pinchPoints).forEach((link1) => {
        if (!structure[link1]) {
            structure[link1] = {};
        }

        Object.keys(steps[0].pinchPoints[link1]).forEach((link2) => {
            structure[link1][link2] = 0;
            tempAnimatedPinchPoints[link1 + "___" + link2] = {
                position: { x: [], y: [], z: [] },
                scale: { x: [], y: [], z: [] },
                color: { r: [], g: [], b: [] },
            };
        });
    });

    let timesteps = steps.map((step) => step.time);
    steps.forEach((step) => {
        Object.keys(step.pinchPoints).forEach((link1) => {
            Object.keys(step.pinchPoints[link1]).forEach((link2) => {
                tempAnimatedPinchPoints[link1 + "___" + link2].position.x.push(
                    step.pinchPoints[link1][link2].position.x
                );
                tempAnimatedPinchPoints[link1 + "___" + link2].position.y.push(
                    step.pinchPoints[link1][link2].position.y
                );
                tempAnimatedPinchPoints[link1 + "___" + link2].position.z.push(
                    step.pinchPoints[link1][link2].position.z
                );
                tempAnimatedPinchPoints[link1 + "___" + link2].scale.x.push(
                    step.pinchPoints[link1][link2].scale.x
                );
                tempAnimatedPinchPoints[link1 + "___" + link2].scale.y.push(
                    step.pinchPoints[link1][link2].scale.y
                );
                tempAnimatedPinchPoints[link1 + "___" + link2].scale.z.push(
                    step.pinchPoints[link1][link2].scale.z
                );
                tempAnimatedPinchPoints[link1 + "___" + link2].color.r.push(
                    step.pinchPoints[link1][link2].color.r
                );
                tempAnimatedPinchPoints[link1 + "___" + link2].color.g.push(
                    step.pinchPoints[link1][link2].color.g
                );
                tempAnimatedPinchPoints[link1 + "___" + link2].color.b.push(
                    step.pinchPoints[link1][link2].color.b
                );
            });
        });
    });

    const animatedPinchPoints = map(
        tempAnimatedPinchPoints,
        (pinchPoint) => ({
            frame: "world",
            position: {
                x: interpolateScalar(timesteps, pinchPoint.position.x),
                y: interpolateScalar(timesteps, pinchPoint.position.y),
                z: interpolateScalar(timesteps, pinchPoint.position.z),
            },
            scale: {
                x: interpolateScalar(timesteps, pinchPoint.scale.x),
                y: interpolateScalar(timesteps, pinchPoint.scale.y),
                z: interpolateScalar(timesteps, pinchPoint.scale.z),
            },
            color: {
                r: interpolateScalar(timesteps, pinchPoint.color.r),
                g: interpolateScalar(timesteps, pinchPoint.color.g),
                b: interpolateScalar(timesteps, pinchPoint.color.b),
                a: 0.3,
            },
        })
    );
    return animatedPinchPoints;
};

export function pinchpointAnimationFromExecutable(robotAgent, stepData) {
    let pairedLinks = robotAgent.properties.pinchPointPairLinks;

    let steps = [{ time: 0, pinchPoints: {} }];
    pairedLinks.forEach((pair) => {
        if (!steps[0].pinchPoints[pair["link1"]]) {
            steps[0].pinchPoints[pair["link1"]] = {};
        }
        steps[0].pinchPoints[pair["link1"]][pair["link2"]] = {
            scale: { x: 0, y: 0, z: 0 },
            position: { x: 0, y: 0, z: 0 },
            color: { r: 0, g: 0, b: 0, a: 0 },
        };
    });

    let currentTime = 0;
    let prevStep = lodash.cloneDeep(steps[steps.length - 1]);
    let previousDistances = {};

    stepData.forEach((step) => {
        let formattedProxData = likProximityAdjustment(
            robotAgent ? robotAgent.properties.pinchPointPairLinks : [],
            step.data?.proximity,
            false
        );
        prevStep.pinchPoints = {
            ...prevStep.pinchPoints,
            ...pinchPointVisualsByStep(
                pairedLinks,
                formattedProxData,
                previousDistances
            ),
        };
        prevStep.time = step.time;
        steps.push(prevStep);
        previousDistances = lodash.cloneDeep(formattedProxData);
        prevStep = lodash.cloneDeep(steps[steps.length - 1]);
        currentTime = step.time;
    });
    steps.push({
        ...lodash.cloneDeep(steps[steps.length - 1]),
        time: currentTime + 500,
    });

    return stepsToAnimatedPinchPoints(steps);
}

export const occupancyOverlap = (position, occupancyZones) => {
    let overlap = false;
    let zones = Object.values(occupancyZones).filter(
        (v) => v.type === "zoneType"
    );
    for (let i = 0; i < zones.length; i++) {
        //.forEach(zone => {
        let zone = zones[i];
        const xOverlap =
            position.x < zone.properties.position.x + zone.properties.scale.x / 2 &&
            position.x > zone.properties.position.x - zone.properties.scale.x / 2;
        const yOverlap =
            position.y < zone.properties.position.z + zone.properties.scale.z / 2 &&
            position.y > zone.properties.position.z - zone.properties.scale.z / 2;
        if (xOverlap && yOverlap) {
            overlap = true;
        }
    }
    return overlap;
};

export function poseToColor(pose, frame, focused, occupancyZones) {
    let color = { r: 255, g: 255, b: 255, a: focused ? 1 : 0 };
    let pos = pose.refData
        ? pose.refData.properties.position
        : pose.properties.position;
    if (frame === "safety" && occupancyOverlap(pos, occupancyZones)) {
        color.r = 233;
        color.g = 53;
        color.b = 152;
    } else if (frame === "performance" && !pose.reachable) {
        color.r = 204;
        color.g = 75;
        color.b = 10;
    }
    return color;
}

export function poseDataToShapes(pose, frame, occupancyZones) {
    let pose_stored = pose;
    return [
        {
            uuid: `${pose_stored.id}-tag`,
            frame: "world",
            name: pose.name,
            shape: pose_stored.type.includes("location") ? "flag" : "tag",
            position: pose_stored.refData
                ? pose_stored.refData.properties.position
                : pose_stored.properties.position,
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            scale: { x: -0.25, y: 0.25, z: 0.25 },
            highlighted: false,
            showName: false,
            color: poseToColor(pose_stored, frame, false, occupancyZones),
        },
        {
            uuid: `${pose_stored.id}-pointer`,
            frame: "world",
            shape: pose_stored.type.includes("location")
                ? "package://app/meshes/LocationMarker.stl"
                : "package://app/meshes/OpenWaypointMarker.stl",
            position: pose_stored.refData
                ? pose_stored.refData.properties.position
                : pose_stored.properties.position,
            rotation: pose_stored.refData
                ? pose_stored.refData.properties.rotation
                : pose_stored.properties.rotation,
            scale: { x: 1, y: 1, z: 1 },
            highlighted: false,
            showName: false,
            color: poseToColor(pose_stored, frame, false, occupancyZones),
        },
    ];
}

export function itemTransformMethod(state, id) {
    let idIncluded = false;
    let transformMethod = "inactive";

    // This variable determines whether the encountered translate/rotate applies to the item being searched
    let isTransformActive = false;

    state.focus.some((f) => {
        let focusItemType = state.programData[f]?.type;
        let focusTypeInfo = focusItemType
            ? Object.keys(state.programSpec.objectTypes[focusItemType].properties)
            : null;
        // Reset the isTransformActive, as the next translate/rotate would reference this new object
        if (focusTypeInfo && idIncluded && focusTypeInfo.includes("position")) {
            isTransformActive = false;
        }

        // Item exists in the focus array
        if (f === id) {
            idIncluded = true;
            isTransformActive = true;
        }

        // If item exists, and still has the potential focus of the translate/rotate, determine if translation/rotation applies
        if (idIncluded && isTransformActive && f === "translate") {
            transformMethod = "translate";
            return true;
        }
        if (idIncluded && isTransformActive && f === "rotate") {
            transformMethod = "rotate";
            return true;
        }
        return false;
    });

    return transformMethod;
}

export function stepsToAnimation(state, tfs, items) {
    // Tracks all things and links for use in the animation
    let dict = {};

    // Array of all timestep values from the compiled data
    let timesteps = [];

    // Tracks the last index of timesteps that a given object (thing or link) was updated
    let lastTimestamp = {};

    // Last timestep - used for adding an addition 500ms delay at the end of the program
    let finalTime = 0;

    // Tracks lists of things indexed by their spawned type (example: all "blade"s are in a list indexed by "blade")
    let trackedByType = {};

    // Array of all tracking thing ids - used to know whether a given id is a thing or not
    let thingList = [];

    // Tracks what is currently grasped by the gripper
    let currentGraspedThingID = '';

    let graspAngle = null;

    // Tracks the previous grasped object by the gripper
    let previousGraspedThingID = '';

    // Tracks whether we are currently in a moveGripper action
    let inMoveGripper = false;

    // Stores the last update for the moveGripper action
    let lastMoveGripperData = {};

    // Create an updatable model of the environment usng the program data
    // This is updated from the compiled data as it's encountered
    let programModel = createEnvironmentModel(state.programData);

    // Gripper offset ID
    const grippers = Object.values(state.programData).filter(v => v.type === 'gripperType');
    let gripOffsetID = grippers.length > 0 ? grippers[0].id + '-gripOffset' : null;

    // Find what is the focus within CoFrame
    let focusStub = null;
    // Only back up to once for the animation correlating to the issue (if applicable)
    if (!state.programData[state.activeFocus]) {
        focusStub =
            state.programData[state.focus[state.focus.length - 2]]?.properties
                ?.compiled;
    } else {
        focusStub = state.programData[state.activeFocus]?.properties?.compiled;
    }
    const compileKeys = Object.keys(focusStub ? focusStub : {});

    // If too many or too few keys, return
    if (compileKeys.length !== 1) {
        return;
    }

    const compiledKey = compileKeys[0];

    // Build up the movements
    let steps = focusStub[compiledKey]?.steps ? focusStub[compiledKey]?.steps : [];
    steps.forEach((step) => {
        // Thing is created/spawned
        if (step.type === STEP_TYPE.SPAWN_ITEM) {
            // Update time trackers
            if (step.time > finalTime) {
                finalTime = step.time;
            }
            timesteps.push(step.time);

            // Create ID for tracking, and add to array
            let id = generateUuid('thing');
            thingList.push(id);

            // Use the inputOutput (that spawned the thing) as the original position
            let ioPosition = queryWorldPose(programModel, step.data.inputOutput, '');

            // Add thing to the program model
            programModel = addToEnvironModel(programModel, 
                'world', 
                id, 
                ioPosition.position, 
                ioPosition.rotation
                );

            // Add thing grasp points to the program model
            let graspPoints = state.programData[step.data.thing].properties.graspPoints;
            graspPoints.forEach(graspId => {
                let gID = generateUuid('graspPoint');
                programModel = addGraspPointToModel(programModel, 
                    id, 
                    gID, 
                    state.programData[graspId].properties.position, 
                    state.programData[graspId].properties.rotation, 
                    state.programData[graspId].properties.gripDistance
                    );
            })

            // Add thing to the tracking
            if (!(step.data.thing in trackedByType)) {
                trackedByType[step.data.thing] = [{
                    id: id
                }];
            } else {
                trackedByType[step.data.thing].push({
                    id: id
                });
            }

            // Create animation object
            dict[id] = {
                position: { x: [], y: [], z: [] },
                rotation: { x: [], y: [], z: [], w: [] },
                hidden: [],
                mesh: state.programData[step.data.thing]?.properties?.mesh,
                relativeTo: step.data.relativeTo?.id,
                type: step.data.thing
            }

            // Hide thing (backfill data)
            for (let i = 0; i < timesteps.length; i++) {
                dict[id].position.x.push(ioPosition.position.x);
                dict[id].position.y.push(ioPosition.position.y);
                dict[id].position.z.push(ioPosition.position.z);
                dict[id].rotation.x.push(ioPosition.rotation.x);
                dict[id].rotation.y.push(ioPosition.rotation.y);
                dict[id].rotation.z.push(ioPosition.rotation.z);
                dict[id].rotation.w.push(ioPosition.rotation.w);
                dict[id].hidden.push(true);
            }

            // Unhide thing (push latest data)
            lastTimestamp[id] = timesteps.length;
            dict[id].position.x.push(ioPosition.position.x);
            dict[id].position.y.push(ioPosition.position.y);
            dict[id].position.z.push(ioPosition.position.z);
            dict[id].rotation.x.push(ioPosition.rotation.x);
            dict[id].rotation.y.push(ioPosition.rotation.y);
            dict[id].rotation.z.push(ioPosition.rotation.z);
            dict[id].rotation.w.push(ioPosition.rotation.w);
            dict[id].hidden.push(false);
        }

        // Thing is consumed/destroyed
        if (step.type === STEP_TYPE.DESTROY_ITEM) {
            // Update time trackers
            if (step.time > finalTime) {
                finalTime = step.time;
            }
            timesteps.push(step.time);

            // Find and remove the tracked thing
            let bucket = trackedByType[step.data.thing];
            if (bucket.length === 1) {
                delete trackedByType[step.data.thing];
            } else {
                // Remove the item that was most recently tracked
                let lst = trackedByType[step.data.thing];
                let idx = 0;
                for (let i = 0; i < lst.length; i++) {
                    if (lst[i].id === previousGraspedThingID) {
                        idx = i;
                        i = lst.length;
                    }
                }
                trackedByType[step.data.thing].splice(idx, 1);
            }

            // Use whatever the last item to have been grabbed
            let id = previousGraspedThingID;

            // backfill data
            for (let i = lastTimestamp[id]; i < timesteps.length; i++) {
                dict[id].position.x.push(dict[id].position.x[curLength-1]);
                dict[id].position.y.push(dict[id].position.y[curLength-1]);
                dict[id].position.z.push(dict[id].position.z[curLength-1]);
                dict[id].rotation.x.push(dict[id].rotation.x[curLength-1]);
                dict[id].rotation.y.push(dict[id].rotation.y[curLength-1]);
                dict[id].rotation.z.push(dict[id].rotation.z[curLength-1]);
                dict[id].rotation.w.push(dict[id].rotation.w[curLength-1]);
                dict[id].hidden.push(dict[id].hidden[curLength-1]);
            }

            // push latest data (hide thing)
            lastTimestamp[id] = timesteps.length;
            dict[id].position.x.push(step.data.position.x);
            dict[id].position.y.push(step.data.position.y);
            dict[id].position.z.push(step.data.position.z);
            dict[id].rotation.x.push(step.data.rotation.x);
            dict[id].rotation.y.push(step.data.rotation.y);
            dict[id].rotation.z.push(step.data.rotation.z);
            dict[id].rotation.w.push(step.data.rotation.w);
            dict[id].hidden.push(false);
        }

        if (step.type === STEP_TYPE.SCENE_UPDATE) {
            // Update time
            if (step.time > finalTime) {
                finalTime = step.time;
            }
            
            // Store last copy of the the moveGripper action
            // This allows us to not constantly update grasped thing's position until the final moment
            // at which it is considered "grasped".
            if (state.programData[step.source] && state.programData[step.source].type === 'moveGripperType') {
                if (!inMoveGripper) {
                    inMoveGripper = true;
                }
                
                lastMoveGripperData = {...step};
            }

            // Once the moveGripper action is done, use the last piece of it's data to update the grasped object.
            if (inMoveGripper && !(state.programData[step.source] && state.programData[step.source].type === 'moveGripperType')) {
                inMoveGripper = false;

                let moveGripper = state.programData[lastMoveGripperData.source];
                let grasping = false;
                let releasing = false;

                // Get the corresponding bucket of things that could be potentially grasped
                let thing = lastMoveGripperData.data.thing.id ? lastMoveGripperData.data.thing.id : lastMoveGripperData.data.thing;
                let bucket = trackedByType[thing];

                // Grasped item is a tool, not a thing
                if (!bucket && state.programData[thing]?.type === 'toolType') {
                    trackedByType[thing] = [{
                        id: thing
                    }];
                    bucket = trackedByType[thing];

                    // Add tool grasp points to the program model
                    let graspPoints = state.programData[thing].properties.graspPoints;
                    graspPoints.forEach(graspId => {
                        let gID = generateUuid('graspPoint');
                        programModel = addGraspPointToModel(programModel, 
                            thing, 
                            gID, 
                            state.programData[graspId].properties.position, 
                            state.programData[graspId].properties.rotation, 
                            state.programData[graspId].properties.gripDistance
                            );
                    });

                    let toolPos = queryWorldPose(programModel, thing, '');

                    // Create animation object
                    dict[thing] = {
                        position: { x: [], y: [], z: [] },
                        rotation: { x: [], y: [], z: [], w: [] },
                        hidden: [],
                        mesh: state.programData[thing]?.properties?.mesh,
                        relativeTo: state.programData[thing]?.properties.relativeTo?.id,
                        type: thing
                    }

                    // backfill data
                    let posStart = lastTimestamp[thing] ? lastTimestamp[thing] : 0;
                    for (let i = posStart; i < timesteps.length; i++) {
                        dict[thing].position.x.push(toolPos.position.x);
                        dict[thing].position.y.push(toolPos.position.y);
                        dict[thing].position.z.push(toolPos.position.z);
                        dict[thing].rotation.x.push(toolPos.rotation.x);
                        dict[thing].rotation.y.push(toolPos.rotation.y);
                        dict[thing].rotation.z.push(toolPos.rotation.z);
                        dict[thing].rotation.w.push(toolPos.rotation.w);
                    }
                    lastTimestamp[thing] = timesteps.length;
                }

                let id = '';

                // Update model positions of all links in the gripper
                Object.keys(lastMoveGripperData.data.links).forEach((link) => {
                    programModel = updateEnvironModel(programModel, link, lastMoveGripperData.data.links[link].position, lastMoveGripperData.data.links[link].rotation);
                });

                // Get the gripper offset position/rotation
                let gripperOffset = queryWorldPose(programModel, gripOffsetID, '');
                let gripperRotation = new Quaternion(gripperOffset.rotation.x, gripperOffset.rotation.y, gripperOffset.rotation.z, gripperOffset.rotation.w);
                let selectedGraspRotation = null;

                // If grasp succeeded and we have things available to be grasped, search for the corresponding
                // thing that is being grasped
                if (bucket && moveGripper.properties.positionEnd < moveGripper.properties.positionStart) {
                    for (let i = 0; i < bucket.length; i++) {
                        // Get all potential grasp locations for a given thing
                        let graspPointIDs = getAllChildrenFromModel(programModel, bucket[i].id);
                        // Search over the grasp locations and determine whether any are within some
                        // tolerance of the gripper's offset position/rotation
                        if (graspPointIDs) {
                            graspPointIDs.forEach(graspPointID => {
                                let graspPosition = queryWorldPose(programModel, graspPointID, '');
                                let graspWidth = getUserDataFromModel(programModel, graspPointID, 'width');
                                let graspRotation = new Quaternion(graspPosition.rotation.x, graspPosition.rotation.y, graspPosition.rotation.z, graspPosition.rotation.w);
                                if (!grasping &&
                                    distance(graspPosition.position, gripperOffset.position) <= MAX_GRIPPER_DISTANCE_DIFF &&
                                    graspRotation.angleTo(gripperRotation) <= MAX_GRIPPER_ROTATION_DIFF &&
                                    moveGripper.properties.positionEnd <= graspWidth
                                ) {
                                    grasping = true;
                                    id = bucket[i].id;
                                    selectedGraspRotation = graspRotation;
                                }
                            });
                        }
                    }
                // moveGripper is releasing instead of grasping
                } else if (moveGripper.properties.positionEnd > moveGripper.properties.positionStart &&
                    moveGripper.properties.positionEnd > width) {
                        releasing = true;
                }

                // Update trackers for grasped things
                if (grasping && !releasing) {
                    currentGraspedThingID = id;
                    let {_, rotation} = queryWorldPose(programModel, currentGraspedThingID, '');
                    graspAngle = (new Quaternion(rotation.x, rotation.y, rotation.z, rotation.w)).multiply(selectedGraspRotation.invert());
                }
                if (!grasping && releasing) {
                    previousGraspedThingID = currentGraspedThingID;
                    currentGraspedThingID = '';
                    graspAngle = null;
                }
            }

            timesteps.push(step.time);

            Object.keys(step.data.links).forEach((link) => {
                // Update the program model for each link
                if (link in tfs) {
                    programModel = updateEnvironModel(programModel, link, step.data.links[link].position, step.data.links[link].rotation);
                }

                // If link didn't previously exist add it
                if (!dict[link]) {
                    dict[link] = {
                        position: { x: [], y: [], z: [] },
                        rotation: { x: [], y: [], z: [], w: [] },
                    };
                    lastTimestamp[link] = 0;
                }

                // First time encountering link (but t iterations have passed since program beginning)
                // so use first data piece to backfill information up to the current time
                if (
                    lastTimestamp[link] === 0 &&
                    lastTimestamp[link] + 1 !== timesteps.length
                ) {
                    for (let i = 0; i < timesteps.length; i++) {
                        dict[link].position.x.push(step.data.links[link].position.x);
                        dict[link].position.y.push(step.data.links[link].position.y);
                        dict[link].position.z.push(step.data.links[link].position.z);
                        dict[link].rotation.x.push(step.data.links[link].rotation.x);
                        dict[link].rotation.y.push(step.data.links[link].rotation.y);
                        dict[link].rotation.z.push(step.data.links[link].rotation.z);
                        dict[link].rotation.w.push(step.data.links[link].rotation.w);
                    }
                // Link exists in dict and t interactions have passed since we previously updated it
                // Use the last data point to backfill up to the current time
                } else if (
                    lastTimestamp[link] !== 0 &&
                    lastTimestamp[link] + 1 !== timesteps.length
                ) {
                    let posLength = dict[link].position.x.length;

                    for (let tmpLength = posLength; tmpLength < timesteps.length; tmpLength++) {
                        dict[link].position.x.push(dict[link].position.x[posLength - 1]);
                        dict[link].position.y.push(dict[link].position.y[posLength - 1]);
                        dict[link].position.z.push(dict[link].position.z[posLength - 1]);
                        dict[link].rotation.x.push(dict[link].rotation.x[posLength - 1]);
                        dict[link].rotation.y.push(dict[link].rotation.y[posLength - 1]);
                        dict[link].rotation.z.push(dict[link].rotation.z[posLength - 1]);
                        dict[link].rotation.w.push(dict[link].rotation.w[posLength - 1]);
                        if (thingList.includes(link)) {
                            dict[link].hidden.push(dict[link].hidden[posLength-1]);
                        }
                    }
                }

                // Add newest data point
                lastTimestamp[link] = timesteps.length;
                dict[link].position.x.push(step.data.links[link].position.x);
                dict[link].position.y.push(step.data.links[link].position.y);
                dict[link].position.z.push(step.data.links[link].position.z);
                dict[link].rotation.x.push(step.data.links[link].rotation.x);
                dict[link].rotation.y.push(step.data.links[link].rotation.y);
                dict[link].rotation.z.push(step.data.links[link].rotation.z);
                dict[link].rotation.w.push(step.data.links[link].rotation.w);
            });

            // Update gripped object
            if (currentGraspedThingID !== '') {
                // Get world pose of the gripper offset and update the grasped object to this pose
                let gripperOffset = queryWorldPose(programModel, gripOffsetID, '');
                // Rotate the thing to use the grasped point's rotation (relative to the thing)
                let adjustedRotation = graspAngle ?  new Quaternion(gripperOffset.rotation.x, gripperOffset.rotation.y, gripperOffset.rotation.z, gripperOffset.rotation.w).multiply(graspAngle) : gripperOffset.rotation
                let rotatedToGraspPoint = {x: adjustedRotation.x, y: adjustedRotation.y, z: adjustedRotation.z, w: adjustedRotation.w};
                programModel = updateEnvironModel(programModel, currentGraspedThingID, gripperOffset.position, rotatedToGraspPoint);

                // If thing hasn't been updated to the most recent timestep, backfill data using last data point
                if (lastTimestamp[currentGraspedThingID] !== 0 &&
                    lastTimestamp[currentGraspedThingID] + 1 !== timesteps.length) {
                    let posLength = dict[currentGraspedThingID].position.x.length;

                    for (let tmpLength = posLength; tmpLength < timesteps.length; tmpLength++) {
                        dict[currentGraspedThingID].position.x.push(dict[currentGraspedThingID].position.x[posLength - 1]);
                        dict[currentGraspedThingID].position.y.push(dict[currentGraspedThingID].position.y[posLength - 1]);
                        dict[currentGraspedThingID].position.z.push(dict[currentGraspedThingID].position.z[posLength - 1]);
                        dict[currentGraspedThingID].rotation.x.push(dict[currentGraspedThingID].rotation.x[posLength - 1]);
                        dict[currentGraspedThingID].rotation.y.push(dict[currentGraspedThingID].rotation.y[posLength - 1]);
                        dict[currentGraspedThingID].rotation.z.push(dict[currentGraspedThingID].rotation.z[posLength - 1]);
                        dict[currentGraspedThingID].rotation.w.push(dict[currentGraspedThingID].rotation.w[posLength - 1]);
                        dict[currentGraspedThingID].hidden.push(dict[currentGraspedThingID].hidden[posLength - 1]);
                    }
                }

                // Add newest datapoint
                lastTimestamp[currentGraspedThingID] = timesteps.length;
                dict[currentGraspedThingID].position.x.push(gripperOffset.position.x);
                dict[currentGraspedThingID].position.y.push(gripperOffset.position.y);
                dict[currentGraspedThingID].position.z.push(gripperOffset.position.z);
                dict[currentGraspedThingID].rotation.x.push(rotatedToGraspPoint.x);
                dict[currentGraspedThingID].rotation.y.push(rotatedToGraspPoint.y);
                dict[currentGraspedThingID].rotation.z.push(rotatedToGraspPoint.z);
                dict[currentGraspedThingID].rotation.w.push(rotatedToGraspPoint.w);
                dict[currentGraspedThingID].hidden.push(false);
            }
        // Else the step type isn't an update/spawn/destroy, so update the time and buffer all object positions and rotations
        } else if (timesteps.length > 0) {
            // Update time trackers
            timesteps.push(step.time);
            if (step.time > finalTime) {
                finalTime = step.time;
            }

            // Buffer all object position and rotations
            Object.keys(dict).forEach((link) => {
                let posLength = dict[link].position.x.length;

                for (let tmpLength = posLength; tmpLength < timesteps.length; tmpLength++) {
                    dict[link].position.x.push(dict[link].position.x[posLength - 1]);
                    dict[link].position.y.push(dict[link].position.y[posLength - 1]);
                    dict[link].position.z.push(dict[link].position.z[posLength - 1]);
                    dict[link].rotation.x.push(dict[link].rotation.x[posLength - 1]);
                    dict[link].rotation.y.push(dict[link].rotation.y[posLength - 1]);
                    dict[link].rotation.z.push(dict[link].rotation.z[posLength - 1]);
                    dict[link].rotation.w.push(dict[link].rotation.w[posLength - 1]);
                    if (thingList.includes(link)) {
                        dict[link].hidden.push(dict[link].hidden[posLength-1]);
                    }
                }

                lastTimestamp[link] = timesteps.length;
            });
        }
    });

    // Iterate through final objects and buffer out times (add additional 500ms)
    finalTime += 500;
    timesteps.push(finalTime);

    // Buffer all final position/rotations for objects and then interpolate the data for robot-scene to animate
    Object.keys(dict).forEach((link) => {
        let posLength = dict[link].position.x.length;
        // Buffer all relevant data for the object
        for (let tmpLength = posLength; tmpLength < timesteps.length; tmpLength++) {
            dict[link].position.x.push(dict[link].position.x[posLength - 1]);
            dict[link].position.y.push(dict[link].position.y[posLength - 1]);
            dict[link].position.z.push(dict[link].position.z[posLength - 1]);
            dict[link].rotation.x.push(dict[link].rotation.x[posLength - 1]);
            dict[link].rotation.y.push(dict[link].rotation.y[posLength - 1]);
            dict[link].rotation.z.push(dict[link].rotation.z[posLength - 1]);
            dict[link].rotation.w.push(dict[link].rotation.w[posLength - 1]);
            if (thingList.includes(link)) {
                dict[link].hidden.push(dict[link].hidden[posLength-1]);
            }
        }

        // If object already exists in the scene data, update the tf with the interpolated data
        if (link in tfs) {
            tfs[link].position = {
                x: interpolateScalar(timesteps, dict[link].position.x),
                y: interpolateScalar(timesteps, dict[link].position.y),
                z: interpolateScalar(timesteps, dict[link].position.z),
            };
            tfs[link].rotation = {
                x: interpolateScalar(timesteps, dict[link].rotation.x),
                y: interpolateScalar(timesteps, dict[link].rotation.y),
                z: interpolateScalar(timesteps, dict[link].rotation.z),
                w: interpolateScalar(timesteps, dict[link].rotation.w),
            };
        // If object is in the thinglist, create the tf and item for it
        } else if (thingList.includes(link)) {
            tfs[link] = {
                frame: 'world',
                position: {
                    x: interpolateScalar(timesteps, dict[link].position.x),
                    y: interpolateScalar(timesteps, dict[link].position.y),
                    z: interpolateScalar(timesteps, dict[link].position.z),
                },
                rotation: {
                    x: interpolateScalar(timesteps, dict[link].rotation.x),
                    y: interpolateScalar(timesteps, dict[link].rotation.y),
                    z: interpolateScalar(timesteps, dict[link].rotation.z),
                    w: interpolateScalar(timesteps, dict[link].rotation.w),
                },
                scale: { x:1, y:1, z:1 }
            }
            items[link] = {
                shape: dict[link].mesh,
                frame: link,
                position: { x: 0, y: 0, z: 0 },
                rotation: { w: 1, x: 0, y: 0, z: 0 },
                scale: { x: 1, y: 1, z: 1 },
                transformMode: "inactive",
                color: { r: 0, g: 200, b: 0, a: 0.2 },
                highlighted: false,
                hidden: interpolateScalar(timesteps, dict[link].hidden)
            }
        }
    });
}
