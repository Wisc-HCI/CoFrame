// import { objectMap } from "./helpers";
import {
    poseDataToShapes,
    DEFAULT_WAYPOINT_COLOR,
    DEFAULT_LOCATION_COLOR,
    UNREACHABLE_COLOR,
    OCCUPANCY_ERROR_COLOR,
    occupancyOverlap,
    DEFAULT_TRAJECTORY_COLOR,
    executablePrimitives,
    tfAnimationFromExecutable,
    robotFramesFromPose,
    machineDataToPlaceholderPreviews,
    PINCH_POINT_FIELDS,
    pinchpointAnimationFromExecutable
} from './helpers';
import debounce from 'lodash.debounce';
// import throttle from 'lodash.throttle';
import { COLLISION_MESHES, EVD_MESH_LOOKUP } from './initialSim';

export const computedSlice = (state) => {
    const ROBOT_PARTS = Object.keys(state.programData).filter(v => v.includes('robot'));
    const GRIPPER_PARTS = Object.keys(state.programData).filter(v => v.includes('gripper'));

    let executablePrimitives = {};

    // ===================== TFs =====================
    let tfs = {};
    let items = {};

    Object.values(state.programData).filter(v => v.type === 'thingType').forEach(thing => {
        // for now, place the things at origin. 
        // console.log(thing)
        tfs[thing.id] = {
            frame: 'world',
            translation: { x: 0, y: 0, z: 0 },
            rotation: { w: 1, x: 0, y: 0, z: 0 }
        }
    })
    // Object.values(state.data.regions).forEach(region=>{
    //     tfs[region.uuid] = {
    //         frame:region.link,
    //         translation:region.center_position,
    //         rotation:region.center_orientation
    //     }
    // })

    if (state.focusItem.uuid) {
        const executable = executablePrimitives[state.focusItem.uuid];
        if (executable) {
            tfs = tfAnimationFromExecutable(executable, tfs)
        } //else if (state.data[state.focusItem.uuid].frames) {
        //  console.log(state.data[state.focusItem.uuid].frames)
        //    tfs = { ...tfs, ...robotFramesFromPose(state.data[state.focusItem.uuid]) }
        //}
    }

    // Pull world based tf data from the machines and robot links
    Object.values(state.programData).filter(v => v.type === 'tfType').forEach(data => {
        tfs[data.id] = {
            frame: data.properties.frame,
            translation: data.properties.position,
            rotation: data.properties.rotation
        }
    });

    // ===================== Items =====================

    const focusedTrajectoryChildren = state.focusItem.type === 'trajectory' ? [
        state.programData[state.focusItem.uuid].properties.startLocation,
        ...state.programData[state.focusItem.uuid].properties.waypoints,
        state.programData[state.focusItem.uuid].properties.endLocation,
    ] : []

    // Add items from the initial static scene
    Object.values(state.programData).filter(v => v.type === 'linkType' || v.type === 'fixtureType').forEach(item => {
        const itemKey = item.id;
        let highlighted = false;
        let meshObject = state.programData[item.properties.mesh];
        let collisionObject = item.properties.collision ? state.programData[item.properties.collision] : null;
        if (ROBOT_PARTS.indexOf(state.focusItem.uuid) >= 0 && ROBOT_PARTS.indexOf(itemKey) >= 0) {
            highlighted = true
        } else if (GRIPPER_PARTS.indexOf(state.focusItem.uuid) >= 0 && GRIPPER_PARTS.indexOf(itemKey) >= 0) {
            highlighted = true
        } else if (state.focusItem.uuid === itemKey || state.secondaryFocusItem.uuid === itemKey) {
            highlighted = true
        }
        items[itemKey] = {
            shape: meshObject.properties.keyword,
            name: item.name,
            frame: item.properties.frame,
            position: meshObject.properties.position,
            rotation: meshObject.properties.rotation,
            color: meshObject.properties.color,
            scale: meshObject.properties.scale,
            transformMode: "inactive",
            highlighted,
            onClick: (e) => {
                if (
                    state.focusItem.transformMode === "translate" ||
                    state.focusItem.transformMode === "rotate" ||
                    state.secondaryFocusItem.transformMode === "translate" ||
                    state.secondaryFocusItem.transformMode === "rotate"
                ) {

                } else {
                    console.log('clicked ' + item.name)
                    e.stopPropagation();
                    state.setFocusItem('scene', item.id);
                }

            },
            onMove: (transform) => { console.log(transform) }
        }

        if (collisionObject) {
            items[itemKey + '-collision'] = {
                shape: COLLISION_MESHES[collisionObject.properties.keyword] ? COLLISION_MESHES[collisionObject.properties.keyword] : collisionObject.properties.keyword,
                name: item.name + ' Collision',
                frame: item.properties.frame,
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
    })

    // Add occupancy zones
    Object.values(state.programData).forEach(entry => {
        if (entry.type === 'zoneType' && state.programData[entry.properties.agent].type === 'humanAgentType') {
            items[entry.id] = {
                shape: 'cube',
                name: entry.name,
                frame: 'world',
                position: entry.properties.position,
                rotation: entry.properties.orientation,
                color: { ...OCCUPANCY_ERROR_COLOR, a: 0.2 },
                scale: entry.properties.scale,
                transformMode: "inactive",
                highlighted: false,
                onClick: (_) => { },
                hidden: !state.occupancyVisible
            }
        } else if (entry.type === 'machineType') {
            // console.log(entry);
            // console.log(entry.name)
            // const mesh = EVD_MESH_LOOKUP[entry.mesh]
            let entryProps = entry.ref ? state.programData[entry.ref].properties : entry.properties;
            let meshObject = state.programData[entryProps.mesh];
            let collisionObject = state.programData[entryProps.collisionMesh];

            items[entry.id] = {
                shape: meshObject.properties.keyword,
                name: entry.name,
                frame: entryProps.tf,
                position: meshObject.properties.position,
                rotation: meshObject.properties.rotation,
                scale: meshObject.properties.scale,
                transformMode: 'inactive',
                highlighted: state.focusItem.uuid === entry.id,
                onClick: (e) => {
                    if (state.focusItem.transformMode === "translate" || state.focusItem.transformMode === "rotate") {

                    } else {
                        console.log('clicked ' + entry.name)
                        e.stopPropagation();
                        state.setFocusItem('data', entry.id);
                    }

                },
            }
            // Now add collisions
            items[entry.id + '-collision'] = {
                shape: collisionObject.properties.keyword,
                name: entry.name + ' Collision',
                frame: entryProps.frame,
                position: collisionObject.properties.position,
                rotation: collisionObject.properties.position,
                scale: collisionObject.properties.scale,
                transformMode: 'inactive',
                highlighted: false,
                color: { r: 250, g: 0, b: 0, a: 0.6 },
                wireframe: true,
                hidden: !state.collisionsVisible,
                onClick: (_) => { },
            }

            // Now enumerate the inputs and render the zones and items.
            // if (state.focusItem.uuid === machine.uuid) {
            //     Object.keys(machine.inputs).forEach(thingType=>{
            //         machine.inputs[thingType].forEach(zoneInfo=>{
            //             items[zoneInfo.region_uuid].color.r = 200;
            //             items[zoneInfo.region_uuid].hidden = false;
            //         })
            //     })
            //     Object.keys(machine.outputs).forEach(thingType=>{
            //         machine.outputs[thingType].forEach(zoneInfo=>{
            //             items[zoneInfo.region_uuid].color.g = 200;
            //             items[zoneInfo.region_uuid].hidden = false;
            //         })
            //     })
            // }

            //items = { ...items, ...machineDataToPlaceholderPreviews(machine, state.data.thingTypes, state.data.regions, state.data.placeholders) }
        } else if (entry.type === 'locationType' || entry.type === 'waypointType') {
            const focused = state.focusItem.uuid === entry.id || state.secondaryFocusItem.uuid === entry.id;
            const trajectoryFocused = focusedTrajectoryChildren.includes(entry.id);
            let correctEntry = entry.ref ? state.programData[entry.ref] : entry;

            // Handle in the case where the trajectory is focused
            let color = entry.type === 'location' ? { ...DEFAULT_LOCATION_COLOR } : { ...DEFAULT_WAYPOINT_COLOR };
            //console.log(item.joints.reachable);
            if (state.frame === 'performance' && !correctEntry.properties.reachable) {//pose, frame, focused, locationOrWaypoint
                //console.log("entered");
                color = { ...UNREACHABLE_COLOR };
            } else if (state.frame === 'safety' && occupancyOverlap(correctEntry.properties.position, state.programData)) {
                color = { ...OCCUPANCY_ERROR_COLOR };
            }
            if (trajectoryFocused) {
                const idx = focusedTrajectoryChildren.indexOf(entry.id);
                color.a = (time) => 0.5 * Math.pow(Math.E, -Math.sin(time / 800 + idx * 0.98));
            }

            const debouncedOnMove = debounce(transform => {
                state.setPoseTransform(entry.id, transform);
                state.requestJointProcessorUpdate('location', entry.id)
            }, 1000)

            // poseDataToShapes(entry, state.frame, state.programData).forEach((shape) => {
            //     items[shape.uuid] = {
            //         ...shape,
            //         highlighted: focused,
            //         hidden: !focused && !trajectoryFocused,
            //         color,
            //         transformMode: shape.uuid.includes('pointer') && focused ? state.focusItem.transformMode : "inactive",
            //         onMove: shape.uuid.includes('pointer') && focused && state.focusItem.transformMode !== 'inactive' ? debouncedOnMove : (_) => { }
            //     };
            //     // e => setItemProperty('location', location_uuid, 'position', { ...state.data.locations[location_uuid].position, x: e[0], y: e[1], z: e[2] }); 


            // })
        }
    })

    // Pinch Point visualizations
    if (state.issues[state.secondaryFocusItem.uuid] && state.executablePrimitives[state.focusItem.uuid] && state.issues[state.secondaryFocusItem.uuid].code === 'pinchPoints') {
        console.log('---------show pinchpoints---------')
        const pinchPointAnimations = pinchpointAnimationFromExecutable(state.executablePrimitives[state.focusItem.uuid])
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

    // Add regions
    // Object.values(state.data.regions).forEach(region=>{
    //     const debouncedOnMove = debounce(transform=>{
    //         state.setRegionTransform(region.uuid,transform);
    //     },1000)
    //     items[region.uuid] = {
    //         shape: region.uncertainty_x ? 'cube' : 'sphere',
    //         frame: region.uuid,
    //         position: {x:0,y:0,z:0},
    //         rotation: {w:1,x:0,y:0,z:0},
    //         scale: region.uncertainty_x ? {x:region.uncertainty_x*5,y:region.uncertainty_y*5,z:region.uncertainty_z*5} : {x:region.uncertainty_radius*5,y:region.uncertainty_radius*5,z:region.uncertainty_radius*5},
    //         transformMode: state.secondaryFocusItem.uuid === region.uuid ? state.secondaryFocusItem.transformMode : 'inactive',
    //         color:{r:0,g:0,b:0,a:0.3},
    //         highlighted: state.secondaryFocusItem.uuid === region.uuid,
    //         hidden: true,
    //         onClick: (_)=>{},
    //         onMove: debouncedOnMove
    //     }
    // })

    // if (state.focusItem.type === 'machine') {
    //     Object.values(state.data.machines[state.focusItem.uuid].inputs).forEach(regionInfo=>{
    //         regionInfo.forEach(input=>focusedInputs.push(input.region_uuid))
    //     });
    //     Object.values(state.data.machines[state.focusItem.uuid].outputs).forEach(regionInfo=>{
    //         regionInfo.forEach(output=>focusedOutputs.push(output.region_uuid))
    //     });
    // }

    // ===================== Lines =====================
    let lines = {};

    Object.values(executablePrimitives).forEach(ePrim => {
        if (ePrim) {
            Object.values(ePrim).forEach(primitive => {
                if (primitive.type === "move-trajectory") {
                    const hidden = state.focusItem.uuid !== primitive.uuid && state.secondaryFocusItem.uuid !== primitive.uuid;
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

    // Object.values(state.programData).filter(v => v.type === 'trajectoryType').forEach(trajectory => {
    //     const hidden = state.focusItem.uuid !== trajectory.uuid && state.secondaryFocusItem.uuid !== trajectory.uuid;
    //     let poses = []
    //     if (trajectory.properties.startLocation) {
    //         poses.push(state.programData[trajectory.properties.startLocation])
    //     }
    //     trajectory.properties.waypoints.forEach(waypointId => {
    //         poses.push(state.programData[waypointId])
    //     })
    //     if (trajectory.properties.endLocation) {
    //         poses.push(state.programData[trajectory.properties.endLocation])
    //     }
    //     const vertices = poses.map(pose => {
    //         let color = { ...DEFAULT_TRAJECTORY_COLOR };
    //         //console.log(item.joints.reachable);
    //         if (state.frame === 'performance' && !pose.joints.reachable) {//pose, frame, focused, locationOrWaypoint
    //             //console.log("entered");
    //             color = { ...UNREACHABLE_COLOR };
    //         } else if (state.frame === 'safety' && occupancyOverlap(pose.properties.position, state.programData)) {
    //             color = { ...OCCUPANCY_ERROR_COLOR };
    //         }
    //         return {
    //             position: pose.properties.position,
    //             color
    //         }
    //     })
    //     lines[trajectory.id] = { name: trajectory.name, vertices, frame: 'world', hidden, width: 2 }
    // })

    // ===================== Hulls =====================
    let hulls = {}

    Object.values(executablePrimitives).forEach(ePrim => {
        if (ePrim) {
            Object.values(ePrim).forEach(primitive => {
                if (primitive.type === "node.primitive.move-trajectory.") {
                    const hidden = state.focusItem.uuid !== primitive.uuid && state.secondaryFocusItem.uuid !== primitive.uuid;
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
        hulls
    })
}