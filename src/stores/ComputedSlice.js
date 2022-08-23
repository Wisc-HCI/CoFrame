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
import { STEP_TYPE } from './Constants';
import { filter } from "lodash";
import { createEnvironmentModel, queryWorldPose, updateEnvironModel } from '../helpers/geometry';

export const computedSlice = (state) => {
    let executablePrimitives = {};

    // ===================== TFs =====================
    let tfs = {};
    let items = {};
    let texts = {};

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

            //console.log()


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
        } else if (entry.type === 'zoneType' && state.programData[entry.properties.agent].type === 'humanAgentType') {
            items[entry.id] = {
                shape: 'cube',
                name: entry.name,
                frame: entry.properties.relativeTo ? entry.properties.relativeTo : 'world',
                position: entry.properties.position,
                rotation: entry.properties.rotation,
                color: { ...OCCUPANCY_ERROR_COLOR, a: 0.2 },
                scale: entry.properties.scale,
                transformMode: itemTransformMethod(state, entry.id),
                highlighted: false,
                hidden: !state.occupancyVisible
            }
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
                })
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
                })
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
            })
        } else if (entry.type === 'robotAgentType' || entry.type === 'humanAgentType' || entry.type === 'gripperType') {
            tfs[entry.id] = {
                frame: entry.properties.relativeTo ? entry.properties.relativeTo : "world",
                position: entry.properties.position,
                rotation: entry.properties.rotation,
                scale: { x: 1, y: 1, z: 1 }
            }
            // if (entry.type === 'gripperType') {

            //     items[entry.id+'-gripperOffset'] = {
            //         shape: 'arrow',
            //         name: `${entry.name} Gripper Position`,
            //         frame:entry.id,
            //         position: entry.properties.gripPositionOffset,
            //         rotation: entry.properties.gripRotationOffset,
            //         scale: {x:0.14,y:0.07,z:0.05},
            //         color: { r: 0, g: 250, b: 250, a: 0.9 },
            //     }
            //     items[entry.id+'-gripperGoal'] = {
            //         shape: 'arrow',
            //         name: `${entry.name} Gripper Goal Position`,
            //         frame: entry.properties.relativeTo,
            //         position: {x:0,y:0,z:0},
            //         rotation: {x:0,y:0,z:0,w:1},
            //         scale: {x:0.14,y:0.07,z:0.05},
            //         color: { r: 250, g: 250, b: 0, a: 0.9 },
            //     }                       
            // }
            //items = { ...items, ...machineDataToPlaceholderPreviews(machine, state.data.thingTypes, state.data.regions, state.data.placeholders) }
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
            if (trajectoryFocused) {
                const idx = focusedTrajectoryChildren.indexOf(entry.id);
                color.a = (time) => 0.5 * Math.pow(Math.E, -Math.sin(time / 800 + idx * 0.98));
            } else {
                color.a = 0.7
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
                // e => setItemProperty('location', location_uuid, 'position', { ...state.data.locations[location_uuid].position, x: e[0], y: e[1], z: e[2] });
            })


            // items[entry.id+'-poseRepresentation'] = {
            //     shape: 'arrow',
            //     name: `${entry.name} Gripper Position Goal`,
            //     frame:'world',
            //     position: entry.properties.position,
            //     rotation: entry.properties.rotation,
            //     scale: {x:0.14,y:0.07,z:0.05},
            //     color: { r: 0, g: 250, b: 250, a: 0.6 },
            // }

            // const baseGripPose =  entry.properties.compiled[JSON.stringify(['root'])]?.goalPose;
            // console.log(baseGripPose)
            // if (baseGripPose) {
            //     items[entry.id+'-gripperOffsetBase'] = {
            //         shape: 'arrow',
            //         name: `${entry.name} Gripper Position Goal`,
            //         frame:'world',
            //         position: baseGripPose.position,
            //         rotation: baseGripPose.rotation,
            //         scale:{x:0.14,y:0.07,z:0.05},
            //         color: { r: 250, g: 250, b: 0, a: 0.6 },
            //     }
            // }
        }
        // } else if (entry.type === 'graspPointType') {
        //     let entryProps = entry.properties;
        //     tfs[entry.id] = {
        //         frame: entry.properties.relativeTo ? entry.properties.relativeTo : "world",
        //         position: entry.properties.position,
        //         rotation: entry.properties.rotation,
        //         transformMode: itemTransformMethod(state, entry.id),
        //         scale: {x:1,y:1,z:1}
        //     }

        //     const gpData = state.programData[gp];
        //     items['tool-viz-' + gp] = {
        //         frame: entry.id,
        //         shape: "package://app/meshes/LocationMarker.stl",
        //         position: gpData.properties.position,
        //         rotation: gpData.properties.rotation,
        //         scale: { x: 1, y: 1, z: 1 },
        //         highlighted: false,
        //         showName: false,
        //         color: { r: 0, g: 200, b: 0, a: 0.2 },
        //         hidden: !state.focus.includes(entry.id)
        //     }
        // }
    })

    // Pinch Point visualizations
    if (deepestIssue && deepestIssue.code === 'pinchPoints' && state.programData[deepestIssue.focus[0]]) {
        const pinchPointAnimations = pinchpointAnimationFromExecutable(Object.values(state.programData).filter(v => v.type === "robotAgentType")[0], state.programData[deepestIssue.focus[0]].properties.compiled["{}"].steps)
        Object.keys(pinchPointAnimations).forEach(field => {
            items[field] = {
                shape: 'sphere',
                rotation: { w: 1, x: 0, y: 0, z: 0 },
                ...pinchPointAnimations[field]
            }
        });
    }

    // ===================== Lines =====================
    let lines = {};

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
            poses.push(state.programData[trajectory.properties.startLocation])
        }
        trajectory.properties.waypoints.forEach(waypointId => {
            poses.push(state.programData[waypointId])
        })
        if (trajectory.properties.endLocation) {
            poses.push(state.programData[trajectory.properties.endLocation])
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
        })

        let program = filter(state.programData, function (v) { return v.type === 'programType' && v.dataType === DATA_TYPES.INSTANCE })[0];
        let steps = program.properties.compiled["{}"]?.steps;
        let sceneTmp = (steps && moveTrajectoryId) ? steps.filter(step => step.type === STEP_TYPE.SCENE_UPDATE && step.source === moveTrajectoryId) : [];
        let programModel = createEnvironmentModel(state.programData);
        let gripOffsetID = Object.values(state.programData).filter(v => v.type === 'gripperType')[0].id + '-gripOffset';
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
    })

    // ===================== Hulls =====================
    let hulls = {}

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
    })

    stepsToAnimation(state, tfs, items);











    return ({
        executablePrimitives,
        tfs,
        items,
        lines,
        hulls,
        texts
    })
}
