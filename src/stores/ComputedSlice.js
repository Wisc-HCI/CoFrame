// import { objectMap } from "./helpers";
import {
    poseDataToShapes,
    DEFAULT_WAYPOINT_COLOR,
    DEFAULT_LOCATION_COLOR,
    UNREACHABLE_COLOR,
    OCCUPANCY_ERROR_COLOR,
    occupancyOverlap,
    DEFAULT_TRAJECTORY_COLOR,
    // tfAnimationFromExecutable,
    pinchpointAnimationFromExecutable,
    itemTransformMethod,
    stepsToAnimation
} from './helpers';
// import throttle from 'lodash.throttle';
// import { COLLISION_MESHES, EVD_MESH_LOOKUP } from './initialSim';
import { DATA_TYPES } from 'simple-vp/dist/components';
// import { filter } from "lodash";

export const computedSlice = (state) => {
    let executablePrimitives = {};

    // ===================== TFs =====================
    let tfs = {};
    let items = {};
    let texts = {};

    Object.values(state.programData).filter(v => v.type === 'thingType' && v.dataType === DATA_TYPES.INSTANCE).forEach(thing => {
        // for now, place the things at origin. 
        // console.log(thing)
        tfs[thing.id] = {
            frame: 'world',
            position: { x: 0, y: 0, z: 0 },
            rotation: { w: 1, x: 0, y: 0, z: 0 },
            scale: {x:1,y:1,z:1}
        }
    })

    let reversedFocus = [];
    for (var i = state.focus.length - 1; i >= 0; i--) {
        reversedFocus.push(state.focus[i]);
    }
    
    // Show the tf animation of the farthest-down focus
    reversedFocus.some(f=>{
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
    reversedFocus.some(f=>{
        if (state.issues[f]) {
            deepestIssue = state.issues[f]
            return true
        } else {
            return false
        }
    })

    // ===================== Items =====================
    let focusedTrajectoryChildren = [];
    state.focus.forEach((entry) => {
        // let trajectory = null;
        if (state.programData[entry]?.type === "moveTrajectoryType" || state.programData[entry]?.type === "trajectoryType") {
            let trajectory = state.programData[entry];
            if (state.programData[entry]?.type === "moveTrajectoryType") {
                trajectory = state.programData[state.programData[entry].properties.trajectory];
            }
            // let trajectory = state.programData[entry];
            if (state.programData[trajectory.properties.startLocation]?.ref && state.programData[trajectory.properties.endLocation]?.ref) {
                focusedTrajectoryChildren.push(state.programData[trajectory.properties.startLocation].ref);
                trajectory.properties.waypoints.forEach((wp) => {
                    focusedTrajectoryChildren.push(state.programData[wp].ref);
                });
                focusedTrajectoryChildren.push(state.programData[trajectory.properties.endLocation].ref);
            }
        }
    });

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
                scale: {x:1,y:1,z:1}
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
                items[input] = {
                    shape: thing.properties.mesh,
                    frame: inputObj.properties.relativeTo ? inputObj.properties.relativeTo : "world",
                    position: inputObj.properties.position,
                    rotation: inputObj.properties.rotation,
                    scale: {x:0.2,y:0.2,z:0.2},
                    transformMode: itemTransformMethod(state, input),
                    color: {r:0,g:200,b:0,a:0.2},
                    highlighted: false,
                    hidden: !state.focus.includes(entry.id)
                }
            });   
            entry.properties.outputs.forEach(output => {
                let outputObj = state.programData[output];
                let thing = state.programData[outputObj.properties.thing];
                items[output] = {
                    shape: thing.properties.mesh,
                    frame: outputObj.properties.relativeTo ? outputObj.properties.relativeTo : "world",
                    position: outputObj.properties.position,
                    rotation: outputObj.properties.rotation,
                    scale: {x:0.2,y:0.2,z:0.2},
                    transformMode: itemTransformMethod(state, output),
                    color: {r:0,g:200,b:0,a:0.2},
                    highlighted: false,
                    hidden: !state.focus.includes(entry.id)
                }
            });            
        } else if (entry.type === 'machineType' || entry.type === 'toolType') {
            let entryProps = entry.properties;
            let meshObject = state.programData[entryProps.mesh];
            let collisionObject = state.programData[entryProps.collision];
            tfs[entry.id] = {
                frame: entry.properties.relativeTo ? entry.properties.relativeTo : "world",
                position: entry.properties.position,
                rotation: entry.properties.rotation,
                transformMode: itemTransformMethod(state, entry.id),
                scale: {x:1,y:1,z:1}
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
                    transformMode: 'inactive',
                    highlighted: false,
                    color: { r: 250, g: 0, b: 0, a: 0.6 },
                    wireframe: true,
                    hidden: !state.collisionsVisible
                }
            });

            //items = { ...items, ...machineDataToPlaceholderPreviews(machine, state.data.thingTypes, state.data.regions, state.data.placeholders) }
        } else if (entry.type === 'robotAgentType' || entry.type === 'humanAgentType' || entry.type === 'gripperType') {
            tfs[entry.id] = {
                frame: entry.properties.relativeTo ? entry.properties.relativeTo : "world",
                position: entry.properties.position,
                rotation: entry.properties.rotation,
                scale: {x:1,y:1,z:1}
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
    })

    // Pinch Point visualizations
    if (deepestIssue && deepestIssue.code === 'pinchPoints' && state.programData[deepestIssue.focus.id]) {
        console.log('---------show pinchpoints---------')
        const pinchPointAnimations = pinchpointAnimationFromExecutable(Object.values(state.programData).filter(v => v.type === "robotAgentType")[0], state.programData[deepestIssue.focus.id].properties.compiled["{}"].steps)
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

    Object.values(executablePrimitives).forEach(ePrim => {
        if (ePrim) {
            Object.values(ePrim).forEach(primitive => {
                if (primitive.type === "move-trajectory") {
                    const hidden = !state.focus.includes(primitive.uuid);
                    if (state.secondaryFocusItem.type === "issue") {
                        const currentIssue = state.issues[state.secondaryFocusItem.uuid];
                        if (currentIssue && currentIssue.sceneData && currentIssue.sceneData.vertices) {
                            let vertKeys = Object.keys(currentIssue.sceneData.vertices);
                            for (let i = 0; i < vertKeys.length; i++) {
                                lines[primitive.uuid.concat(vertKeys[i])] = { name: vertKeys[i], vertices: currentIssue.sceneData.vertices[vertKeys[i]], frame: 'world', hidden, width: 4 };
                            }
                        }
                    }
                }
            });
        }
    });

    Object.values(state.programData).filter(v => v.type === 'trajectoryType' && v.dataType === DATA_TYPES.INSTANCE).forEach(trajectory => {
        let inMoveTrajectory = false;
        state.focus.forEach(focusItem => {
            let obj = state.programData[focusItem];
            if (obj?.type === "moveTrajectoryType" && obj?.properties?.trajectory === trajectory.id) {
                inMoveTrajectory = true;
            }
        })
        const hidden = !state.focus.includes(trajectory.id) && !inMoveTrajectory;
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
        lines[trajectory.id] = { name: trajectory.name, vertices, frame: 'world', hidden, width: 2 }
    })

    // ===================== Hulls =====================
    let hulls = {}

    // Object.values(executablePrimitives).forEach(ePrim => {
    //     if (ePrim) {
    //         Object.values(ePrim).forEach(primitive => {
    //             if (primitive.type === "node.primitive.move-trajectory.") {
    //                 const hidden = !state.focus.includes(primitive.uuid);
    //                 if (state.secondaryFocusItem.type === "issue") {
    //                     const currentIssue = state.issues[state.secondaryFocusItem.uuid];
    //                     if (currentIssue && currentIssue.sceneData && currentIssue.sceneData.hulls) {
    //                         let vertKeys = Object.keys(currentIssue.sceneData.hulls);
    //                         for (let i = 0; i < vertKeys.length; i++) {
    //                             hulls[primitive.uuid.concat(vertKeys[i])] = { name: vertKeys[i], vertices: currentIssue.sceneData.hulls[vertKeys[i]].vertices, color: currentIssue.sceneData.hulls[vertKeys[i]].color, frame: 'world', hidden, width: 2 };
    //                         }
    //                     }
    //                 }
    //             }
    //         });
    //     }
    // });

    // Show preview of deepest preview type.
    reversedFocus.some(focusId=>{
        const item = state.programData[focusId];
        if (!item) {
            return false
        } else if (item.type === 'waypointType' || item.type === 'locationType') {
            Object.values(item.properties.states).forEach(robotGroup=>{
                    Object.values(robotGroup).forEach(gripperGroup=>{
                        console.log('CURRENT TFS', tfs)
                        console.log('ADDED LINKS',gripperGroup.links)
                        tfs = {...tfs, ...gripperGroup.links}
                    })
                }
            )
            return true
        } else {
            return false
        }
    })

    stepsToAnimation(state, tfs);

    return ({
        executablePrimitives,
        tfs,
        items,
        lines,
        hulls,
        texts
    })
}