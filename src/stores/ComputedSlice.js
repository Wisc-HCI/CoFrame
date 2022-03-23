// import { objectMap } from "./helpers";
import {
    poseDataToShapes,
    DEFAULT_WAYPOINT_COLOR,
    DEFAULT_LOCATION_COLOR,
    UNREACHABLE_COLOR,
    OCCUPANCY_ERROR_COLOR,
    occupancyOverlap,
    DEFAULT_TRAJECTORY_COLOR,
    tfAnimationFromExecutable,
    PINCH_POINT_FIELDS,
    pinchpointAnimationFromExecutable,
    itemTransformMethod
} from './helpers';
import lodash from 'lodash';
// import throttle from 'lodash.throttle';
import { COLLISION_MESHES, EVD_MESH_LOOKUP } from './initialSim';
import { DATA_TYPES } from 'simple-vp/dist/components';

export const computedSlice = (state) => {
    const ROBOT_PARTS = Object.keys(state.programData).filter(v => v.includes('robot'));
    const GRIPPER_PARTS = Object.keys(state.programData).filter(v => v.includes('gripper'));

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
            translation: { x: 0, y: 0, z: 0 },
            rotation: { w: 1, x: 0, y: 0, z: 0 }
        }
    })

    let reversedFocus = [];
    for (var i = state.focus.length - 1; i >= 0; i--) {
        reversedFocus.push(state.focus[i]);
    }
    
    // Show the tf animation of the farthest-down focus
    reversedFocus.some(f=>{
        if (executablePrimitives[f]) {
            tfs = tfAnimationFromExecutable(executable, tfs)
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

    // const focusedTrajectoryChildren = state.focusItem.type === 'trajectory' ? [
    //     state.programData[state.focusItem.uuid].properties.startLocation,
    //     ...state.programData[state.focusItem.uuid].properties.waypoints,
    //     state.programData[state.focusItem.uuid].properties.endLocation,
    // ] : []
    const focusedTrajectoryChildren = [];

    // Add items from the initial static scene
    Object.values(state.programData).filter(v => v.dataType === DATA_TYPES.INSTANCE).forEach(entry => {
        if (entry.type === 'linkType' || entry.type === 'fixtureType') {
            const itemKey = entry.id;
            let highlighted = false;
            let meshObject = state.programData[entry.properties.mesh];
            let collisionObject = entry.properties.collision ? state.programData[entry.properties.collision] : null;
            if (lodash.intersection(ROBOT_PARTS,state.focus).length > 0 && ROBOT_PARTS.indexOf(itemKey) >= 0) {
                highlighted = true
            } else if (lodash.intersection(GRIPPER_PARTS,state.focus).length > 0 && GRIPPER_PARTS.includes(itemKey)) {
                highlighted = true
            }

            tfs[entry.id] = {
                frame: entry.properties.relativeTo ? entry.properties.relativeTo : "world",
                translation: entry.properties.position,
                rotation: entry.properties.rotation
            }

            if (meshObject) {
                items[itemKey] = {
                    shape: meshObject.properties.keyword,
                    name: entry.name,
                    frame: entry.id,
                    position: meshObject.properties.position,
                    rotation: meshObject.properties.rotation,
                    color: meshObject.properties.color,
                    scale: meshObject.properties.scale,
                    transformMode: itemTransformMethod(state, entry.id),
                    highlighted,
                    onClick: (e) => {
                        if (!state.focus.includes('translate') && !state.focus.includes('rotate')) {
                            console.log('clicked ' + entry.name)
                            e.stopPropagation();
                            state.addFocusItem(entry.id,false);
                        }
        
                    },
                    onMove: (transform) => { console.log(transform) }
                }
            }

            if (collisionObject) {
                items[itemKey + '-collision'] = {
                    shape: COLLISION_MESHES[collisionObject.properties.keyword] ? COLLISION_MESHES[collisionObject.properties.keyword] : collisionObject.properties.keyword,
                    name: entry.name + ' Collision',
                    frame: entry.id,
                    position: collisionObject.properties.position,
                    rotation: collisionObject.properties.rotation,
                    scale: collisionObject.properties.scale,
                    color: { r: 250, g: 0, b: 0, a: 0.6 },
                    transformMode: "inactive",
                    highlighted: false,
                    wireframe: true,
                    hidden: !state.collisionsVisible,
                    onClick: (_) => { },
                    onMove: (transform) => { console.log(transform) }
                };
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
                onClick: (_) => { },
                hidden: !state.occupancyVisible
            }
        } else if (entry.type === 'processType') {
            entry.properties.inputs.forEach(input => {
                let inputObj = state.programData[input];
                let thing = state.programData[inputObj.properties.thing];
                items[input] = {
                    shape: EVD_MESH_LOOKUP[thing.properties.mesh],
                    frame: inputObj.properties.relativeTo ? inputObj.properties.relativeTo : "world",
                    position: inputObj.properties.position,
                    rotation: inputObj.properties.rotation,
                    scale: {x:0.2,y:0.2,z:0.2},
                    transformMode: itemTransformMethod(state, input),
                    color: {r:0,g:200,b:0,a:0.2},
                    highlighted: false,
                    hidden: !state.focus.includes(entry.id),
                    onClick: (_)=>{console.log(input)}
                }
            });   
            entry.properties.outputs.forEach(output => {
                let outputObj = state.programData[output];
                let thing = state.programData[outputObj.properties.thing];
                items[output] = {
                    shape: EVD_MESH_LOOKUP[thing.properties.mesh],
                    frame: outputObj.properties.relativeTo ? outputObj.properties.relativeTo : "world",
                    position: outputObj.properties.position,
                    rotation: outputObj.properties.rotation,
                    scale: {x:0.2,y:0.2,z:0.2},
                    transformMode: itemTransformMethod(state, output),
                    color: {r:0,g:200,b:0,a:0.2},
                    highlighted: false,
                    hidden: !state.focus.includes(entry.id),
                    onClick: (_)=>{console.log(output)}
                }
            });            
        } else if (entry.type === 'machineType' || entry.type === 'toolType') {
            let entryProps = entry.properties;
            let meshObject = state.programData[entryProps.mesh];
            let collisionObject = state.programData[entryProps.collisionMesh];
            tfs[entry.id] = {
                frame: entry.properties.relativeTo ? entry.properties.relativeTo : "world",
                translation: entry.properties.position,
                rotation: entry.properties.rotation
            }
            items[entry.id] = {
                shape: meshObject.properties.keyword,
                name: entry.name,
                frame: entry.id,
                position: meshObject.properties.position,
                rotation: meshObject.properties.rotation,
                scale: meshObject.properties.scale,
                transformMode: itemTransformMethod(state, entry.id),
                highlighted: state.focus.includes(entry.id),
                onClick: (e) => {
                    if (!state.focus.includes('translate') && !state.focus.includes('rotate')) {
                        console.log('clicked ' + entry.name)
                        e.stopPropagation();
                        state.addFocusItem(entry.id,false);
                    }

                },
            }
            // Now add collisions
            items[entry.id + '-collision'] = {
                shape: collisionObject.properties.keyword,
                name: entry.name + ' Collision',
                frame: entry.id,
                position: collisionObject.properties.position,
                rotation: collisionObject.properties.rotation,
                scale: collisionObject.properties.scale,
                transformMode: 'inactive',
                highlighted: false,
                color: { r: 250, g: 0, b: 0, a: 0.6 },
                wireframe: true,
                hidden: !state.collisionsVisible,
                onClick: (_) => { },
            }

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
            }

            const onMove = (transform) => {
                state.setPoseTransform(entry.id,transform)
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
                    transformMode: shape.uuid.includes('pointer') && focused ? transform : "inactive",
                    onMove: shape.uuid.includes('pointer') && focused && transform !== 'inactive' ? onMove : (_) => { }
                };
                // e => setItemProperty('location', location_uuid, 'position', { ...state.data.locations[location_uuid].position, x: e[0], y: e[1], z: e[2] });
            })
        }
    })

    // Pinch Point visualizations
    if (deepestIssue && deepestIssue.code === 'pinchPoints' && executablePrimitives[deepestIssue.focus.uuid]) {
        console.log('---------show pinchpoints---------')
        const pinchPointAnimations = pinchpointAnimationFromExecutable(executablePrimitives[deepestIssue.focus.uuid])
        Object.keys(PINCH_POINT_FIELDS).forEach(field => {
            items[field] = {
                shape: 'sphere',
                rotation: { w: 1, x: 0, y: 0, z: 0 },
                onClick: () => { },
                ...pinchPointAnimations[field],
                frame: null//'simulated_'+PINCH_POINT_FIELDS[field].parent
            }
            console.log(field)
            console.log(items[field])
        })
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
        const hidden = !state.focus.includes(trajectory.id);
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
            let pos = pose.refData ? pose.refData.properties.position : pose.properties.position;
            let reachable = pose.refData ? pose.refData.properties.reachable : pose.properties.reachable;
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

    Object.values(executablePrimitives).forEach(ePrim => {
        if (ePrim) {
            Object.values(ePrim).forEach(primitive => {
                if (primitive.type === "node.primitive.move-trajectory.") {
                    const hidden = !state.focus.includes(primitive.uuid);
                    if (state.secondaryFocusItem.type === "issue") {
                        const currentIssue = state.issues[state.secondaryFocusItem.uuid];
                        if (currentIssue && currentIssue.sceneData && currentIssue.sceneData.hulls) {
                            let vertKeys = Object.keys(currentIssue.sceneData.hulls);
                            for (let i = 0; i < vertKeys.length; i++) {
                                hulls[primitive.uuid.concat(vertKeys[i])] = { name: vertKeys[i], vertices: currentIssue.sceneData.hulls[vertKeys[i]].vertices, color: currentIssue.sceneData.hulls[vertKeys[i]].color, frame: 'world', hidden, width: 2 };
                            }
                        }
                    }
                }
            });
        }
    });

    return ({
        executablePrimitives,
        tfs,
        items,
        lines,
        hulls,
        texts
    })
}