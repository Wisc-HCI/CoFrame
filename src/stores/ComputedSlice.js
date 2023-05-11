import {
    DEFAULT_WAYPOINT_COLOR,
    DEFAULT_LOCATION_COLOR,
    UNREACHABLE_COLOR,
    OCCUPANCY_ERROR_COLOR,
    DEFAULT_TRAJECTORY_COLOR
} from './helpers';
import {
    poseDataToShapes,
    occupancyOverlap,
    itemTransformMethod,
    stepsToAnimation,
    pinchpointAnimationFromExecutable
} from '../helpers/computedSlice';
import { DATA_TYPES } from 'simple-vp';
import { GOAL_FUNCTIONS, ROOT_PATH, STEP_TYPE } from './Constants';
import lodash from 'lodash';
import { createEnvironmentModel, queryWorldPose, updateEnvironModel } from '../helpers/geometry';
import { csArrayEquality, objectEquality } from '../helpers/performance';
import useCompiledStore from './CompiledStore';
import useStore from './Store';
import { filter } from "lodash";

const updateRobotScene = (useCompiledStore, useStore) => {
    let executablePrimitives = {};
    let tfs = {};
    let items = {};
    let texts = {};
    let lines = {};
    let hulls = {};

    const state = useStore.getState();
    const compiledState = useCompiledStore.getState();

    let reversedFocus = [];
    for (var i = state.focus.length - 1; i >= 0; i--) {
        reversedFocus.push(state.focus[i]);
    }

    // Show the tf animation of the farthest-down focus
    reversedFocus.some(f => {
        if (executablePrimitives[f]) {
            // TODO: FIX
            // tfs = tfAnimationFromExecutable(executable, tfs)
            return true
        } else {
            return false
        }
    })

    // Get the deepest issue in case we need to visualize things
    let deepestIssue = null;
    let visualizeIssue = false;
    reversedFocus.some(f => {
        if (state.issues[f]) {
            visualizeIssue = true;
            deepestIssue = state.issues[f]
            return true
        } else {
            return false
        }
    })

    const robotAgent = Object.values(state.programData).filter(v => v.type === "robotAgentType")[0];
    const gripperAgent = Object.values(state.programData).filter(v => v.type === "gripperType")[0];

    // ===================== Items =====================
    let focusedTrajectoryChildren = [];
    if (!visualizeIssue) {
        state.focus.forEach((entry) => {

            // let trajectory = null;
            if (state.programData[entry]?.type === "moveTrajectoryType" || state.programData[entry]?.type === "trajectoryType") {
                let trajectoryTmp = null;

                if (state.programData[entry]?.type === "moveTrajectoryType") {
                    trajectoryTmp = state.programData[state.programData[entry].properties.trajectory];
                } else {
                    trajectoryTmp = state.programData[entry]
                }

                let trajectory = trajectoryTmp?.ref ? state.programData[trajectoryTmp.ref] : trajectoryTmp;

                if (trajectory && state.programData[trajectory.properties.startLocation]?.ref && state.programData[trajectory.properties.endLocation]?.ref) {
                    focusedTrajectoryChildren.push(state.programData[trajectory.properties.startLocation].ref);
                    trajectory.properties.waypoints.forEach((wp) => {
                        focusedTrajectoryChildren.push(state.programData[wp].ref);
                    });
                    focusedTrajectoryChildren.push(state.programData[trajectory.properties.endLocation].ref);
                }
            }
        });
    }

    // Add items from the initial static scene
    Object.values(state.programData).filter(v => v.dataType === DATA_TYPES.INSTANCE).forEach(entry => {
        if (entry.type === 'linkType' || entry.type === 'fixtureType') {
            const itemKey = entry.id;
            let highlighted = false;
            let meshObject = state.programData[entry.properties.mesh];
            let collisionObject = entry.properties.collision ? state.programData[entry.properties.collision] : null;
            if (entry.type === 'fixtureType' && state.focus.includes(entry.id)) {
                highlighted = true
            } else if (entry.type === "linkType" && state.focus.includes(entry.properties.agent)) {
                highlighted = true
            }

            tfs[entry.id] = {
                frame: entry.properties.relativeTo ? entry.properties.relativeTo : "world",
                position: entry.properties.position,
                rotation: entry.properties.rotation,
                transformMode: itemTransformMethod(state, entry.id),
                scale: { x: 1, y: 1, z: 1 }
            }

            if (meshObject) {
                items[itemKey] = {
                    shape: meshObject.properties.keyword,
                    name: entry.name,
                    frame: entry.id,
                    position: meshObject.properties.position,
                    rotation: meshObject.properties.rotation,
                    color: meshObject.properties.color,//{r:10,g:10,b:10,a:0.35},//
                    scale: meshObject.properties.scale,
                    highlighted
                }
            }

            if (collisionObject) {
                collisionObject.properties.componentShapes.forEach((shape) => {
                    let componentShape = state.programData[shape];
                    items[entry.id + shape] = {
                        shape: componentShape.properties.keyword,
                        name: componentShape.name,
                        frame: entry.id,
                        position: componentShape.properties.position,
                        rotation: componentShape.properties.rotation,
                        scale: componentShape.properties.scale,
                        color: { r: 250, g: 0, b: 0, a: 0.6 },
                        transformMode: "inactive",
                        highlighted: false,
                        wireframe: true,
                        hidden: !state.collisionsVisible
                    };
                })
            }
        } else if (entry.type === 'zoneType' && state.programData[entry.properties.agent]?.type === 'humanAgentType') {
            tfs[entry.id] = {
                frame: entry.properties.relativeTo ? entry.properties.relativeTo : "world",
                position: entry.properties.position,
                rotation: entry.properties.rotation,
                transformMode: itemTransformMethod(state, entry.id),
                scale: entry.properties.scale
            }

            items[entry.id] = {
                shape: 'cube',
                name: entry.name,
                frame: entry.id,
                position: {x: 0, y: 0, z: 0},
                rotation: {x: 0, y: 0, z: 0, w: 1},
                color: { ...OCCUPANCY_ERROR_COLOR, a: 0.2 },
                scale: {x: 1, y: 1, z: 1},
                highlighted: false,
                hidden: !state.occupancyVisible
            }

            let meshObj = state.programData[entry?.properties?.mesh];
            if (meshObj) {
                items[entry.id + meshObj.id] = {
                    shape: meshObj.properties.keyword,
                    name: meshObj.name,
                    frame: entry.id,
                    position: meshObj.properties.position,
                    rotation: meshObj.properties.rotation,
                    color: { ...OCCUPANCY_ERROR_COLOR, a: 0.5 },
                    scale: meshObj.properties.scale,
                    highlighted: false,
                    hidden: !state.occupancyVisible
                }
            }

            let collisionObject = state.programData[entry.properties.collision];

            // Now add collisions
            collisionObject?.properties.componentShapes.forEach((collisionShapeId) => {
                let collisionShape = state.programData[collisionShapeId];
                items[entry.id + collisionShapeId] = {
                    shape: collisionShape.properties.keyword,
                    name: collisionShape.id,
                    frame: entry.id,
                    position: collisionShape.properties.position,
                    rotation: collisionShape.properties.rotation,
                    scale: collisionShape.properties.scale,
                    extraParams: collisionShape.properties.extraParams,
                    transformMode: 'inactive',
                    highlighted: false,
                    color: { r: 250, g: 0, b: 0, a: 0.6 },
                    wireframe: true,
                    hidden: !(state.collisionsVisible && state.occupancyVisible)
                }
            });
        } else if (entry.type === 'processType') {
            entry.properties.inputs.forEach(input => {

                let inputObj = state.programData[input];
                let thing = state.programData[inputObj.properties.thing];
                tfs[input] = {
                    frame: inputObj.properties.relativeTo ? inputObj.properties.relativeTo : "world",
                    position: inputObj.properties.position,
                    rotation: inputObj.properties.rotation,
                    scale: { x: 1, y: 1, z: 1 },
                    transformMode: itemTransformMethod(state, input)
                }
                let thingMesh = state.programData?.[thing.properties.mesh];
                if (thingMesh) {
                    items[input] = {
                        shape: thingMesh.properties.keyword,
                        frame: input,
                        position: thingMesh.properties.position,
                        rotation: thingMesh.properties.rotation,
                        scale: thingMesh.properties.scale,
                        transformMode: itemTransformMethod(state, input),
                        color: { r: 0, g: 200, b: 0, a: 0.2 },
                        highlighted: false,
                        hidden: !(state.focus.includes(entry.id) || state.focus.includes(input))
                    };
                } else {
                    items[input] = {
                        shape: thing.properties.mesh,
                        frame: input,
                        position: { x: 0, y: 0, z: 0 },
                        rotation: { w: 1, x: 0, y: 0, z: 0 },
                        scale: { x: 1, y: 1, z: 1 },
                        transformMode: null,
                        color: { r: 0, g: 200, b: 0, a: 0.2 },
                        highlighted: false,
                        hidden: !(state.focus.includes(entry.id) || state.focus.includes(input))
                    };
                }
                thing.properties.graspPoints.forEach((gp) => {
                    const gpData = state.programData[gp];
                    const id = input + gp + '-viz';
                    tfs[id] = {
                        frame: input,
                        position: gpData.properties.position,
                        rotation: gpData.properties.rotation,
                        transformMode: itemTransformMethod(state, gp),
                        scale: { x: 1, y: 1, z: 1 }
                    }
                    items[id] = {
                        frame: id,
                        shape: "package://app/meshes/LocationMarker.stl",
                        position: { x: 0, y: 0, z: 0 },
                        rotation: { x: 0, y: 0, z: 0, w: 1 },
                        scale: { x: 1, y: 1, z: 1 },
                        highlighted: false,
                        showName: false,
                        color: { r: 0, g: 200, b: 0, a: 0.2 },
                        hidden: !(state.focus.includes(entry.id) || state.focus.includes(input) || state.focus.includes(gp))
                    }
                });
            });
            entry.properties.outputs.forEach(output => {

                let outputObj = state.programData[output];
                let thing = state.programData[outputObj.properties.thing];
                tfs[output] = {
                    frame: outputObj.properties.relativeTo ? outputObj.properties.relativeTo : "world",
                    position: outputObj.properties.position,
                    rotation: outputObj.properties.rotation,
                    scale: { x: 1, y: 1, z: 1 },
                    transformMode: itemTransformMethod(state, output)
                }


                let thingMesh = state.programData?.[thing.properties.mesh];
                if (thingMesh) {
                    items[output] = {
                        shape: thingMesh.properties.keyword,
                        frame: output,
                        position: thingMesh.properties.position,
                        rotation: thingMesh.properties.rotation,
                        scale: thingMesh.properties.scale,
                        transformMode: itemTransformMethod(state, output),
                        color: { r: 0, g: 200, b: 0, a: 0.2 },
                        highlighted: false,
                        hidden: !(state.focus.includes(entry.id) || state.focus.includes(output))
                    }
                } else {
                    items[output] = {
                        shape: thing.properties.mesh,
                        frame: output,
                        position: { x: 0, y: 0, z: 0 },
                        rotation: { w: 1, x: 0, y: 0, z: 0 },
                        scale: { x: 1, y: 1, z: 1 },
                        transformMode: itemTransformMethod(state, output),
                        color: { r: 0, g: 200, b: 0, a: 0.2 },
                        highlighted: false,
                        hidden: !(state.focus.includes(entry.id) || state.focus.includes(output))
                    }
                }


                thing.properties.graspPoints.forEach((gp) => {
                    const gpData = state.programData[gp];
                    const id = output + gp + '-viz';
                    tfs[id] = {
                        frame: output,
                        position: gpData.properties.position,
                        rotation: gpData.properties.rotation,
                        transformMode: itemTransformMethod(state, gp),
                        scale: { x: 1, y: 1, z: 1 }
                    }
                    items[id] = {
                        frame: id,
                        shape: "package://app/meshes/LocationMarker.stl",
                        position: { x: 0, y: 0, z: 0 },
                        rotation: { x: 0, y: 0, z: 0, w: 1 },
                        scale: { x: 1, y: 1, z: 1 },
                        highlighted: false,
                        showName: false,
                        color: { r: 0, g: 200, b: 0, a: 0.2 },
                        hidden: !(state.focus.includes(entry.id) || state.focus.includes(output) || state.focus.includes(gp))
                    }
                });
            });
        } else if (entry.type === 'machineType' || entry.type === 'toolType') {
            let entryProps = entry.properties;
            let meshObject = state.programData[entryProps.mesh];
            let collisionObject = state.programData[entryProps.collision];
            let graspPoints = entryProps.graspPoints ? entryProps.graspPoints : [];
            tfs[entry.id] = {
                frame: entry.properties.relativeTo ? entry.properties.relativeTo : "world",
                position: entry.properties.position,
                rotation: entry.properties.rotation,
                transformMode: itemTransformMethod(state, entry.id),
                scale: { x: 1, y: 1, z: 1 }
            }
            items[entry.id] = {
                shape: meshObject.properties.keyword,
                name: entry.name,
                frame: entry.id,
                position: meshObject.properties.position,
                rotation: meshObject.properties.rotation,
                scale: meshObject.properties.scale,
                highlighted: state.focus.includes(entry.id)
            }
            if (meshObject.properties.color.a > 0) {
                items[entry.id]["color"]  = meshObject.properties.color;
            }
            // Now add collisions
            collisionObject?.properties.componentShapes.forEach((collisionShapeId) => {
                let collisionShape = state.programData[collisionShapeId];
                items[entry.id + collisionShapeId] = {
                    shape: collisionShape.properties.keyword,
                    name: collisionShape.id,
                    frame: entry.id,
                    position: collisionShape.properties.position,
                    rotation: collisionShape.properties.rotation,
                    scale: collisionShape.properties.scale,
                    extraParams: collisionShape.properties.extraParams,
                    transformMode: 'inactive',
                    highlighted: false,
                    color: { r: 250, g: 0, b: 0, a: 0.6 },
                    wireframe: true,
                    hidden: !state.collisionsVisible
                }
            });
            graspPoints.forEach((gp) => {
                const gpData = state.programData[gp];
                tfs['tool-viz-' + gp] = {
                    frame: entry.id,
                    position: gpData.properties.position,
                    rotation: gpData.properties.rotation,
                    transformMode: itemTransformMethod(state, gp),
                    scale: { x: 1, y: 1, z: 1 }
                }
                items['tool-viz-' + gp] = {
                    frame: 'tool-viz-' + gp,
                    shape: "package://app/meshes/LocationMarker.stl",
                    position: { x: 0, y: 0, z: 0 },
                    rotation: { x: 0, y: 0, z: 0, w: 1 },
                    scale: { x: 1, y: 1, z: 1 },
                    highlighted: false,
                    showName: false,
                    color: { r: 0, g: 200, b: 0, a: 0.2 },
                    hidden: !(state.focus.includes(entry.id) || state.focus.includes(gp))
                }
            });

            // Ghost grasp points
            graspPoints.forEach((gp) => {
                const gpData = state.programData[gp];
                tfs['ghost--viz--tool--' + gp] = {
                    frame: entry.id,
                    position: gpData.properties.position,
                    rotation: gpData.properties.rotation,
                    transformMode: itemTransformMethod(state, gp),
                    scale: { x: 1, y: 1, z: 1 }
                }
                items['ghost--viz--tool--' + gp] = {
                    frame: 'ghost--viz--tool--' + gp,
                    shape: "package://app/meshes/LocationMarker.stl",
                    position: { x: 0, y: 0, z: 0 },
                    rotation: { x: 0, y: 0, z: 0, w: 1 },
                    scale: { x: 1, y: 1, z: 1 },
                    highlighted: false,
                    showName: false,
                    color: { r: 0, g: 255, b: 0, a: 0.5 },
                    hidden: !(!state.playing && state.activeFocus)
                }
            });
        } else if (entry.type === 'robotAgentType' || entry.type === 'humanAgentType' || entry.type === 'gripperType') {
            tfs[entry.id] = {
                frame: entry.properties.relativeTo ? entry.properties.relativeTo : "world",
                position: entry.properties.position,
                rotation: entry.properties.rotation,
                scale: { x: 1, y: 1, z: 1 }
            }
        } else if (entry.type === 'locationType' || entry.type === 'waypointType') {
            const focused = state.focus.includes(entry.id);
            const trajectoryFocused = focusedTrajectoryChildren.includes(entry.id);
            let correctEntry = entry.ref ? state.programData[entry.ref] : entry;

            // Handle in the case where the trajectory is focused
            let color = entry.type === 'location' ? { ...DEFAULT_LOCATION_COLOR } : { ...DEFAULT_WAYPOINT_COLOR };
            if (state.frame === 'performance' && !correctEntry.properties.reachable) {//pose, frame, focused, locationOrWaypoint
                color = { ...UNREACHABLE_COLOR };
            } else if (state.frame === 'safety' && occupancyOverlap(correctEntry.properties.position, state.programData)) {
                color = { ...OCCUPANCY_ERROR_COLOR };
            }
            let robotColor = {...color};
            if (trajectoryFocused) {
                const idx = focusedTrajectoryChildren.indexOf(entry.id);
                color.a = (time) => 0.5 * Math.pow(Math.E, -Math.sin(time / 800 + idx * 0.98));
                robotColor.a = (time) => 0.2 * Math.pow(Math.E, -Math.sin(time / 800 + idx * 0.98));
            } else {
                color.a = 0.7;
                robotColor.a = 0.35;
            }

            const addItem = (link, isGripper) => {
                let id = 'ghost-' + entry.id + link;
                let linkData = state.programData[link];
                let meshObject = state.programData[linkData.properties.mesh];
                
                let frame = 'world';
                if (!isGripper) {
                    let frameFlag = state.programData[entry.properties.states[robotAgent.id][gripperAgent.id].links[link].frame]?.type === 'robotAgentType';
                    frame = frameFlag ? entry.properties.states[robotAgent.id][gripperAgent.id].links[link].frame : 'ghost-' + entry.id + entry.properties.states[robotAgent.id][gripperAgent.id].links[link].frame
                } else {
                    let frameFlag = state.programData[state.programData[link]?.properties?.relativeTo]?.type === 'gripperType';
                    frame = 'ghost-' + entry.id + (
                        frameFlag ? 
                            state.programData[state.programData[link]?.properties?.relativeTo]?.properties?.relativeTo : 
                            state.programData[link]?.properties?.relativeTo
                        );
                }
                tfs[id] = {
                    frame: frame,
                    position: isGripper ? state.programData[link].properties.position : entry.properties.states[robotAgent.id][gripperAgent.id].links[link].position,
                    rotation: isGripper ? state.programData[link].properties.rotation : entry.properties.states[robotAgent.id][gripperAgent.id].links[link].rotation,
                    transformMode: "inactive",
                    scale: { x: 1, y: 1, z: 1 }
                }
    
                if (meshObject) {
                    items[id] = {
                        shape: meshObject.properties.keyword,
                        name: id,
                        frame: id,
                        position: meshObject.properties.position,
                        rotation: meshObject.properties.rotation,
                        color: robotColor,
                        scale: meshObject.properties.scale,
                        highlighted: false,
                        hidden: !(state.robotPreviewVisible && trajectoryFocused),
                    }
                }
            }

            // Add ghost robot to location
            let addedLinks = false;
            const allLinks = entry.properties?.states?.[robotAgent?.id]?.[gripperAgent?.id]?.links
            if (allLinks) {
                Object.keys(allLinks).forEach(link => {
                    addItem(link, false);
                    addedLinks = true;
                });
            }

            // Add ghost gripper to location
            if (addedLinks) {
                Object.keys(gripperAgent?.properties?.gripperFrames).forEach(link => {
                    addItem(link, true);
                })
            }

            poseDataToShapes(entry, state.frame, state.programData).forEach((shape) => {
                const transform = state.focus.includes('translate')
                    ? 'translate'
                    : state.focus.includes('rotate')
                        ? 'rotate'
                        : 'inactive'
                items[shape.uuid] = {
                    ...shape,
                    highlighted: focused,
                    hidden: !focused && !trajectoryFocused,
                    color,
                    transformMode: shape.uuid.includes('pointer') && focused ? transform : "inactive"
                };
            })
        }
    })

    // Pinch Point visualizations
    if (deepestIssue && deepestIssue.code === 'pinchPoints' && state.programData[deepestIssue.focus[0]]) {
        const pinchPointAnimations = pinchpointAnimationFromExecutable(robotAgent, compiledState[deepestIssue.focus[0]]?.[ROOT_PATH]?.steps)
        Object.keys(pinchPointAnimations).forEach(field => {
            items[field] = {
                shape: 'sphere',
                rotation: { w: 1, x: 0, y: 0, z: 0 },
                ...pinchPointAnimations[field]
            }
        });
    }

    // ===================== Lines =====================
    Object.values(state.programData).filter(v => v.type === 'moveTrajectoryType').forEach(primitive => {
        const hidden = !state.focus.includes(primitive.id);
        if (deepestIssue &&
            state.programData[deepestIssue.focus[0]] &&
            deepestIssue.sceneData &&
            deepestIssue.sceneData.vertices) {

            let vertKeys = Object.keys(deepestIssue.sceneData.vertices);
            for (let i = 0; i < vertKeys.length; i++) {
                lines[primitive.id.concat(vertKeys[i])] = { name: vertKeys[i], vertices: deepestIssue.sceneData.vertices[vertKeys[i]], frame: 'world', hidden, width: 4 };
            }
        }
    });

    Object.values(state.programData).filter(v => v.type === 'trajectoryType' && v.dataType === DATA_TYPES.INSTANCE).forEach(trajectory => {
        let moveTrajectoryId = null;
        state.focus.forEach(focusItem => {
            let obj = state.programData[focusItem];
            if (obj?.type === "moveTrajectoryType" && obj?.properties?.trajectory === trajectory.id) {
                moveTrajectoryId = obj.id;
            }
        });

        const hidden = visualizeIssue || (!state.focus.includes(trajectory.id) && !state.focus.includes(moveTrajectoryId));

        let poses = []
        if (trajectory.properties.startLocation) {
            poses.push(state.programData[trajectory.properties.startLocation]);
        }
        trajectory.properties.waypoints.forEach(waypointId => {
            poses.push(state.programData[waypointId])
        });
        if (trajectory.properties.endLocation) {
            poses.push(state.programData[trajectory.properties.endLocation]);
        }
        const vertices = poses.map(pose => {
            let pos = pose.ref ? state.programData[pose.ref].properties.position : pose.properties.position;
            let reachable = pose.ref ? state.programData[pose.ref].properties.reachable : pose.properties.reachable;
            let color = { ...DEFAULT_TRAJECTORY_COLOR };
            if (state.frame === 'performance' && !reachable) {//pose, frame, focused, locationOrWaypoint
                color = { ...UNREACHABLE_COLOR };
            } else if (state.frame === 'safety' && occupancyOverlap(pos, state.programData)) {
                color = { ...OCCUPANCY_ERROR_COLOR };
            }
            return {
                position: pos,
                color
            }
        });

        let program = filter(state.programData, function (v) { return v.type === 'programType' && v.dataType === DATA_TYPES.INSTANCE })[0];
        let steps = compiledState?.[program.id]?.[ROOT_PATH]?.steps;
        let sceneTmp = (steps && moveTrajectoryId) ? steps.filter(step => step.type === STEP_TYPE.SCENE_UPDATE && step.source === moveTrajectoryId) : [];
        let programModel = createEnvironmentModel(state.programData);
        let gripOffsetID = gripperAgent.id + '-gripOffset';
        let eePoseVerts = sceneTmp.map(sceneUpdate => {
            Object.keys(sceneUpdate.data.links).forEach(link => {
                programModel = updateEnvironModel(programModel, link, sceneUpdate.data.links[link].position, sceneUpdate.data.links[link].rotation);
            });
            let { position, rotation } = queryWorldPose(programModel, gripOffsetID, '');
            return {
                position: position,
                color: { ...DEFAULT_TRAJECTORY_COLOR }
            }
        });
        lines[trajectory.id.concat('-eePose')] = { name: trajectory.name.concat('-eePose'), vertices: eePoseVerts, frame: 'world', hidden, width: 2 };

        lines[trajectory.id] = { name: trajectory.name, vertices, frame: 'world', hidden, width: 2 }
    });

    // ===================== Hulls =====================
    Object.values(state.programData).filter(v => v.type === 'moveTrajectoryType').forEach(primitive => {
        const hidden = !state.focus.includes(primitive.id);
        if (deepestIssue &&
            state.programData[deepestIssue.focus[0]] &&
            deepestIssue.sceneData &&
            deepestIssue.sceneData.hulls) {

            let vertKeys = Object.keys(deepestIssue.sceneData.hulls);
            for (let i = 0; i < vertKeys.length; i++) {
                hulls[primitive.id.concat(vertKeys[i])] = { name: vertKeys[i], vertices: deepestIssue.sceneData.hulls[vertKeys[i]].vertices, color: deepestIssue.sceneData.hulls[vertKeys[i]].color, frame: 'base_link', hidden, width: 2 };
            }
        }
    });

    // Show preview of deepest preview type.
    reversedFocus.some(focusId => {
        const item = state.programData[focusId];
        if (!item) {
            return false
        } else if (item.type === 'waypointType' || item.type === 'locationType') {
            Object.values(item.properties.states).forEach(robotGroup => {
                Object.values(robotGroup).forEach(gripperGroup => {
                    tfs = { ...tfs, ...gripperGroup.links }
                })
            }
            )
            return true
        } else {
            return false
        }
    });

    if (compiledState) {
        stepsToAnimation(state, compiledState, tfs, items);
    }
    
    state.setSceneState({tfs, items, lines, hulls, texts});
}

const executeGoalCondition = (condition, compiledData, programState, programID, otherConditions) => {
    if (condition.type === GOAL_FUNCTIONS.IF) {
        // condition:
        // leftSide:
        // rightSide:
        if (executeGoalCondition(condition.condition, compiledData, programState, programID)) {
            return executeGoalCondition(condition.then, compiledData, programState, programID);
        }
        return executeGoalCondition(condition.else, compiledData, programState, programID);
    } else if (condition.type === GOAL_FUNCTIONS.EQUAL) {
        // leftSide:
        // rightSide:
        return executeGoalCondition(condition.leftSide, compiledData, programState, programID) === executeGoalCondition(condition.rightSide, compiledData, programState, programID);
    } else if (condition.type === GOAL_FUNCTIONS.OR) {
        // leftSide:
        // rightSide:
        return executeGoalCondition(condition.leftSide, compiledData, programState, programID) || executeGoalCondition(condition.rightSide, compiledData, programState, programID);
    } else if (condition.type === GOAL_FUNCTIONS.AND) {
        // leftSide:
        // rightSide:
        return executeGoalCondition(condition.leftSide, compiledData, programState, programID) && executeGoalCondition(condition.rightSide, compiledData, programState, programID);
    } else if (condition.type === GOAL_FUNCTIONS.NOT) {
        // condition:
        return !executeGoalCondition(condition.condition, compiledData, programState, programID);
    } else if (condition.type === GOAL_FUNCTIONS.STR ||
               condition.type === GOAL_FUNCTIONS.INT ||
               condition.type === GOAL_FUNCTIONS.FLOAT ||
               condition.type === GOAL_FUNCTIONS.BOOL) {
        return condition.value;
    } else if (condition.type === GOAL_FUNCTIONS.PID) {
        // path : ["","",...] - path in object
        // id: id of object
        let path = programState.programData[condition.id];
        if (condition.path) {
            condition?.path.forEach(piece => {
                path = path?.[piece];
            })
        }
        return path;
    } else if (condition.type === GOAL_FUNCTIONS.CID) {
        // path : ["","",...] - path in object
        // id: id of object
        let path = compiledData?.[condition.id];
        if (condition.path) {
            condition?.path.forEach(piece => {
                path = path?.[piece];
            })
        }
        return path;
    } else if (condition.type === GOAL_FUNCTIONS.MOVE) {
        // startLocation: id of locationType
        // endLocation: id of locationType
        const stepLength = compiledData?.[programID]?.[ROOT_PATH]?.steps?.length;
        if (stepLength) {
            for (let i = 0; i < stepLength; i++) {
                const step = compiledData?.[programID]?.[ROOT_PATH]?.steps[i];
                const source = programState.programData[step?.source];
                if (step.type === STEP_TYPE.SCENE_UPDATE &&
                    source.type === 'moveTrajectoryType') {
                        const trajectoryData = compiledData?.[source?.properties?.trajectory];
                        if (trajectoryData?.[ROOT_PATH]?.startLocation?.id === condition.startLocation &&
                            trajectoryData?.[ROOT_PATH]?.endLocation?.id === condition.endLocation) {
                                return true;
                        }
                }
            }
        }

        // did not find a trajectory that satisfies the constraints (location1 and location2)
        return false
    } else if (condition.type === GOAL_FUNCTIONS.GRASP) {
        // release: T/F
        // gizmo: id of thingType or toolType
        const stepLength = compiledData?.[programID]?.[ROOT_PATH]?.steps?.length;
        if (stepLength) {
            for (let i = 0; i < stepLength; i++) {
                const step = compiledData?.[programID]?.[ROOT_PATH]?.steps[i];
                const source = programState.programData[step?.source];
                if (step.type === STEP_TYPE.ACTION_START &&
                    source.type === 'moveGripperType' &&
                    step.data.thing.id === condition.gizmo) {
                        // released gizmo
                        if (source.properties.positionEnd < source.properties.positionStart && !condition.release) {
                            return true;
                        }
    
                        // released gizmo
                        if (source.properties.positionEnd > source.properties.positionStart && condition.release) {
                            return true;
                        }
                    }
            }
        }

        // event did not occur
        return false;
    } else if (condition.type === GOAL_FUNCTIONS.PROCESS) {
        // processId: id of the process
        // machineId: id of the machine
        // state: start/wait
        const stepLength = compiledData?.[programID]?.[ROOT_PATH]?.steps?.length;
        if (stepLength) {
            for (let i = 0; i < stepLength; i++) {
                const step = compiledData?.[programID]?.[ROOT_PATH]?.steps[i];
                const source = programState.programData[step?.source];
                if (condition.state === 'start' &&
                    step.type === STEP_TYPE.PROCESS_START &&
                    source.type === 'processStartType' &&
                    step.data.gizmo === condition.machineId &&
                    step.data.process === condition.processId) {
                        return true;
                }
                if (condition.state === 'wait' &&
                    step.type === STEP_TYPE.PROCESS_START &&
                    source.type === 'processWaitType' &&
                    step.data.gizmo === condition.machineId &&
                    step.data.process === condition.processId) {
                        return true;
                }
            }
        }
        
        return false;
    } else if (condition.type === GOAL_FUNCTIONS.CHECKINIT) {
        // machineId: id of the machine
        const stepLength = compiledData?.[programID]?.[ROOT_PATH]?.steps?.length;
        if (stepLength) {
            for (let i = 0; i < stepLength; i++) {
                const step = compiledData?.[programID]?.[ROOT_PATH]?.steps[i];
                const source = programState.programData[step?.source];
                if (step.type === STEP_TYPE.LANDMARK &&
                    source.type === 'machineInitType' &&
                    step.data.machine === condition.machineId) {
                        return true;
                    }
            }
        }

        return false;
    } else if (condition.type === GOAL_FUNCTIONS.ISSUES) {
        // allCategories: T/F
        // categoryList: ["","",...]
        if (condition.allCategories) {
            // Toggle to false if any issue category isn't 0
            let allIssues = programState.reviewableChanges === 0;
            Object.keys(programState.sections).forEach(section => {
                allIssues = allIssues && programState.sections[section].issues.length === 0;
            });
            return allIssues;
        }

        // Toggle to false if any issue category isn't 0
        let allCategories = programState.reviewableChanges === 0;
        condition.categoryList.forEach(category => {
            allCategories = allCategories && programState.sections[category].issues.length === 0;
        });
        return allCategories;
        
    } else if (condition.type === GOAL_FUNCTIONS.CREATE || 
               condition.type === GOAL_FUNCTIONS.DELETE) {
        // programType: ""
        // criteria: [{path:["","",...], value: ANY}]
        const filterByType = Object.values(programState.programData).filter(v => v.type === condition.programType);
        for (let i = 0; i < filterByType.length; i++) {
            const obj = filterByType[i];
            let matchAllCriteria = true;
            for (let j = 0; j < condition.criteria.length; j++) {
                const curCriteria = condition.criteria[j];
                
                let piecedObj = obj;
                for (let k = 0; k < curCriteria.path.length; k++) {
                    piecedObj = piecedObj?.[curCriteria.path[k]];
                }
                
                matchAllCriteria = matchAllCriteria && objectEquality(piecedObj, curCriteria.value)
            }

            // all criteria was matched
            // if this is a create condition, return true, as object was created
            // if this is a delete condition, return false, as the object to delete still exists
            if (matchAllCriteria) {
                return condition.type === GOAL_FUNCTIONS.CREATE;
            }
        }

        // nothing was found to match all criteria
        // so if in deletion, return true - as this implies (near enough) a deletion of the item
        // if this is a create, return false, as the item we wanted was not created
        return condition.type === GOAL_FUNCTIONS.DELETE;
    } else if (condition.type === GOAL_FUNCTIONS.LOADPROGRAM) {
        // Check if all other necessary conditions have been satisfied
        let pass = true;
        condition.goalIdList.forEach(goalId => {
            if (otherConditions[goalId]) {
                pass = pass && otherConditions[goalId]
            } else {
                pass = false;
            }
        });
        return pass;
    }
}

// Subscribes to the compiled store
export const computedSliceCompiledSubscribe = (useCompiledStore, useStore) => {
    const programID = Object.values(useStore.getState().programData).filter((v) => v.type === 'programType')[0].id;
    // const robotAgent = Object.values(useStore.getState().programData).filter(v => v.type === 'robotAgentType')[0];

    // useCompiledStore.subscribe(state => 
    //     [state[programID]?.[ROOT_PATH]?.steps],
    //     (current, previous) => {
    //         let lines = {};
    //         let items = {};
    //         let tfs = {};
    //         const steps = current[0];
    //         const state = useCompiledStore.getState();
    //         const programState = useStore.getState();
    //         const programData = programState.programData;
    //         const trajectories = Object.values(programData).filter(v => v.dataType === DATA_TYPES.INSTANCE && v.type === 'trajectoryType');
    //         const gripperAgent = Object.values(programData).filter(v => v.dataType === DATA_TYPES.INSTANCE && v.type === 'gripperType')[0];
    //         const keys = trajectories.map(t => {return t.id});
    //         const focus = programState.focus;

    //         keys.forEach(key => {
    //             let moveTrajectoryId = null;
    //             const moves = Object.values(programData).filter(v => v.dataType === DATA_TYPES.INSTANCE && v.type === 'moveTrajectoryType');
    //             moves.forEach(move =>{
    //                 if (move.properties?.trajectory === key) {
    //                     moveTrajectoryId = move.id;
    //                 }
    //             })

    //             let sceneTmp = (steps && moveTrajectoryId) ? steps?.filter(step => step.type === STEP_TYPE.SCENE_UPDATE && step.source === moveTrajectoryId) : [];
    //             let programModel = createEnvironmentModel(programData);
    //             let gripOffsetID = gripperAgent.id + '-gripOffset';
    //             const hidden = (!focus.includes(key) && !focus.includes(moveTrajectoryId));
    //             let eePoseVerts = sceneTmp.map(sceneUpdate => {
    //                 Object.keys(sceneUpdate.data.links).forEach(link => {
    //                     programModel = updateEnvironModel(programModel, link, sceneUpdate.data.links[link].position, sceneUpdate.data.links[link].rotation);
    //                 });
    //                 let { position, rotation } = queryWorldPose(programModel, gripOffsetID, '');
    //                 return {
    //                     position: position,
    //                     color: { ...DEFAULT_TRAJECTORY_COLOR }
    //                 }
    //             });
    //             const name = programData[key].name
    //             lines[key.concat('-eePose')] = { name: name.concat('-eePose'), vertices: eePoseVerts, frame: 'world', hidden, width: 2 };  
    //         })


    //         const moveTrajectories = Object.values(programData).filter(v => v.dataType === DATA_TYPES.INSTANCE && v.type === 'moveTrajectoryType');
    //         const moveKeys = moveTrajectories.map(m => {return m.id});
    //         moveKeys.forEach(moveID => {
    //             if (state[moveID]) {
    //                 const pinchPointAnimations = pinchpointAnimationFromExecutable(robotAgent, state[moveID]?.[ROOT_PATH]?.steps);
    //                 Object.values(pinchPointAnimations).forEach(field => {
    //                     items[moveID + field.id] = {
    //                         shape: 'sphere',
    //                         rotation: { w: 1, x: 0, y: 0, z: 0 },
    //                         frame: field.frame,
    //                         position: field.position,
    //                         color: field.color,
    //                         scale: field.scale,
    //                         hidden: true
    //                     }
    //                 });
    //             }
    //         });

    //         programState.applySceneUpdate({tfs, items, lines});
    //     },
    //     { equalityFn: csArrayEquality }
    // );
    const goals = Object.values(useStore.getState().programData).filter((v) => v.type === 'goalType');


    useCompiledStore.subscribe(state =>
        [state?.[programID]?.[ROOT_PATH]?.steps],
        (current, previous) => {
            const programState = useStore.getState();
            updateRobotScene(useCompiledStore, useStore);
            const compiledState = useCompiledStore.getState();
            let conds = {};
            goals.forEach(goal => {
                const pass = executeGoalCondition(goal.properties.condition, compiledState, programState, programID, conds);
                conds[goal.id] = pass;
            });
            programState.updateCompleteGoals(conds);
        },
        { equalityFn: csArrayEquality }
    )
}



export const computedSliceSubscribe = (useStore) => {
    useStore.subscribe(state =>
        [state.programData, state.focus, state.occupancyVisible, state.collisionsVisible, state.robotPreviewVisible, state.playing],
        (current, previous) => {
            updateRobotScene(useCompiledStore, useStore);
        },
        { equalityFn: csArrayEquality }
    )
    
    useStore.subscribe(state => 
        [state.issues, state.reviewableChanges],
        (current, previous) => {
            let conds = {};
            const compiledState = useCompiledStore.getState();
            const programState = useStore.getState();
            const goals = Object.values(programState.programData).filter((v) => v.type === 'goalType');
            const programID = Object.values(programState.programData).filter(v => v.type === 'programType')[0].id;
            goals.forEach(goal => {
                const pass = executeGoalCondition(goal.properties.condition, compiledState, programState, programID, conds);
                conds[goal.id] = pass;
            });
            programState.updateCompleteGoals(conds);
        }, { equalityFn: csArrayEquality })
}