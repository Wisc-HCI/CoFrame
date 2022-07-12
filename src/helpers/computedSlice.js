import lodash from "lodash";
import {
    STEP_TYPE,
} from "../stores/Constants";
import { generateUuid } from "../stores/generateUuid";
import {
    interpolateScalar,
    checkHandThresholds
} from '../stores/helpers';
import { map } from 'lodash';
import { likProximityAdjustment } from "./conversion";

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
    let dict = {};
    let lastTimestamp = {};
    let finalTime = 0;
    let timesteps = [];
    let trackedThings = [];
    let thingList = [];
    // let currentGraspedThing = '';

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
        if (step.type === STEP_TYPE.SPAWN_ITEM) {
            if (step.time > finalTime) {
                finalTime = step.time;
            }

            timesteps.push(step.time);

            // begin tracking thing
            let id = generateUuid('thing');
            thingList.push(id);
            trackedThings.push({
                id: id,
                position: {...step.data.position},
                rotation: {...step.data.rotation},
                relativeTo: step.data.relativeTo?.id,
                type: step.data.thing 
            });

            // create object
            dict[id] = {
                position: { x: [], y: [], z: [] },
                rotation: { x: [], y: [], z: [], w: [] },
                hidden: [],
                mesh: state.programData[step.data.thing]?.properties?.mesh,
                relativeTo: step.data.relativeTo?.id,
            }

            // backfill data
            for (let i = 0; i < timesteps.length; i++) {
                dict[id].position.x.push(step.data.position.x);
                dict[id].position.y.push(step.data.position.y);
                dict[id].position.z.push(step.data.position.z);
                dict[id].rotation.x.push(step.data.rotation.x);
                dict[id].rotation.y.push(step.data.rotation.y);
                dict[id].rotation.z.push(step.data.rotation.z);
                dict[id].rotation.w.push(step.data.rotation.w);
                dict[id].hidden.push(true);
            }

            // push latest data
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
            if (step.time > finalTime) {
                finalTime = step.time;
            }

            timesteps.push(step.time);

            // Add link rotation/position
            Object.keys(step.data.links).forEach((link) => {
                // Link didn't previously exist, so add it
                if (!dict[link]) {
                    dict[link] = {
                        position: { x: [], y: [], z: [] },
                        rotation: { x: [], y: [], z: [], w: [] },
                    };
                    lastTimestamp[link] = 0;
                }
                // If just encountering link (after t iterations) use first data piece to backfill information
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
                    // Use the old data to fill static position until current time
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

                // Add new time
                lastTimestamp[link] = timesteps.length;
                dict[link].position.x.push(step.data.links[link].position.x);
                dict[link].position.y.push(step.data.links[link].position.y);
                dict[link].position.z.push(step.data.links[link].position.z);
                dict[link].rotation.x.push(step.data.links[link].rotation.x);
                dict[link].rotation.y.push(step.data.links[link].rotation.y);
                dict[link].rotation.z.push(step.data.links[link].rotation.z);
                dict[link].rotation.w.push(step.data.links[link].rotation.w);
            });
        } else if (timesteps.length > 0) {
            timesteps.push(step.time);
            if (step.time > finalTime) {
                finalTime = step.time;
            }

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
    // Animate
    finalTime += 500;
    timesteps.push(finalTime);
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
        } else if (thingList.includes(link)) {
            tfs[link] = {
                frame: dict[link].relativeTo ? dict[link].relativeTo : 'world',
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
                scale: { x: 0.2, y: 0.2, z: 0.2 },
                transformMode: "inactive",
                color: { r: 0, g: 200, b: 0, a: 0.2 },
                highlighted: false,
                hidden: interpolateScalar(timesteps, dict[link].hidden)
            }
        }
    });
}