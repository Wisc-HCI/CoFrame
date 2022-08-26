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
import lodash from 'lodash';
import { createEnvironmentModel, queryWorldPose, updateEnvironModel } from '../helpers/geometry';
import shallow from 'zustand/shallow';
import { csArrayEquality } from '../helpers/performance';

// Subscribes to the non-compiled store
export const computedSliceSubscribe = (useStore) => {

    const fixtureAndLink = (update, prev) => {
        const current = update[0];
        const previous = prev[0];
        const keys = Object.keys(current);
        const state = useStore.getState();
        let tfs = {};
        let items = {};
        let different = false;
        let updateProps = false;
        keys.forEach(key => {
            if (previous[key] && (update[1].includes(key) || update[2] !== prev[2])) {
                updateProps = true;
                const entry = current[key];
                let highlighted = false;
                let meshObject = state.programData[entry.properties.mesh];
                let collisionObject = entry.properties.collision ? state.programData[entry.properties.collision] : null;
                if (entry.type === 'fixtureType' && state.focus.includes(key)) {
                    highlighted = true
                } else if (entry.type === "linkType" && state.focus.includes(entry.properties.agent)) {
                    highlighted = true
                }
                tfs[key] = {
                    transformMode: itemTransformMethod(state, entry.id)
                };
        
                if (meshObject) {
                    items[key] = {
                        highlighted
                    }
                }
        
                if (collisionObject) {
                    collisionObject.properties.componentShapes.forEach((shape) => {
                        items[entry.id + shape] = {
                            hidden: !state.collisionsVisible
                        };
                    })
                }
            } else if (!previous[key] || (JSON.stringify(current[key]) !== JSON.stringify(previous[key]))) {
                different = true;
                const entry = current[key];
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
                };
        
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
            }
        });

        if (!updateProps && different) {
            useStore.getState().applySceneUpdate({tfs, items});
        } else if (updateProps) {
            useStore.getState().applyPartialSceneUpdate({tfs, items});
        }
    };

    // Links
    useStore.subscribe(state => 
        [lodash.pickBy(state.programData, (v) => v.dataType === DATA_TYPES.INSTANCE && (v.type === 'linkType')), state.focus, state.collisionsVisible],
        (current, previous) => {
            fixtureAndLink(current, previous);
        },
        { equalityFn: csArrayEquality }
    );

    // Fixture
    useStore.subscribe(state => 
        [lodash.pickBy(state.programData, (v) => v.dataType === DATA_TYPES.INSTANCE && (v.type === 'fixtureType')), state.focus, state.collisionsVisible],
        (current, previous) => {
            fixtureAndLink(current, previous);
        },
        { equalityFn: csArrayEquality }
    );


    const agentAndGripper = (current, previous) => {
        const keys = Object.keys(current);
        let tfs = {};
        let different = false;
        keys.forEach(key => {
          if (!previous[key] || (JSON.stringify(current[key]) !== JSON.stringify(previous[key]))) {
            different = true;
            const entry = current[key];
            tfs[entry.id] = {
              frame: entry.properties.relativeTo ? entry.properties.relativeTo : "world",
              position: entry.properties.position,
              rotation: entry.properties.rotation,
              scale: { x: 1, y: 1, z: 1 }
            }
          }
        })
    
        if (different) {
            useStore.getState().applySceneUpdate({tfs});
        }
    };

    // Robot agent
    useStore.subscribe(state => 
        lodash.pickBy(state.programData, (v) => v.dataType === DATA_TYPES.INSTANCE && (v.type === 'robotAgentType')),
        (current, previous) => {
            agentAndGripper(current, previous);
        },
        { equalityFn: shallow }
    );

    // Human agent
    useStore.subscribe(state => 
        lodash.pickBy(state.programData, (v) => v.dataType === DATA_TYPES.INSTANCE && (v.type === 'humanAgentType')),
        (current, previous) => {
            agentAndGripper(current, previous);
        },
        { equalityFn: shallow }
    );

    // Gripper
    useStore.subscribe(state => 
        lodash.pickBy(state.programData, (v) => v.dataType === DATA_TYPES.INSTANCE && (v.type === 'gripperType')),
        (current, previous) => {
            agentAndGripper(current, previous);
        },
        { equalityFn: shallow }
    );

    const machineAndTool = (update, prev) => {
        const current = update[0];
        const previous = prev[0];

        const keys = Object.keys(current);
        const state = useStore.getState();
        let tfs = {};
        let items = {};
        let different = false;
        let updateProps = false;
        keys.forEach(key => {
            if (previous[key] && (update[1].includes(key) || (update[2] !== prev[2]))) {
                updateProps = true;
                const entry = current[key];
                const entryProps = entry.properties;
                const collisionObject = state.programData[entryProps.collision];
                const graspPoints = entryProps.graspPoints ? entryProps.graspPoints : [];

                tfs[entry.id] = {
                    transformMode: itemTransformMethod(state, entry.id),
                }

                items[entry.id] = {
                    highlighted: state.focus.includes(entry.id)
                }

                // Now add collisions
                collisionObject?.properties.componentShapes.forEach((collisionShapeId) => {
                    items[entry.id + collisionShapeId] = {
                        hidden: !state.collisionsVisible
                    }
                });

                graspPoints.forEach((gp) => {
                    const gpData = state.programData[gp];
                    tfs['tool-viz-' + gp] = {
                        transformMode: itemTransformMethod(state, gp),
                    }
                    items['tool-viz-' + gp] = {
                        hidden: !(state.focus.includes(entry.id) || state.focus.includes(gp))
                    }
                });
            } else if (!previous[key] || (JSON.stringify(current[key]) !== JSON.stringify(previous[key]))) {
                different = true;
                const entry = current[key];
                const entryProps = entry.properties;
                const meshObject = state.programData[entryProps.mesh];
                const collisionObject = state.programData[entryProps.collision];
                const graspPoints = entryProps.graspPoints ? entryProps.graspPoints : [];

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
            }
        });
        
        if (!updateProps && different) {
            useStore.getState().applySceneUpdate({tfs, items});
        } else if (updateProps) {
            useStore.getState().applyPartialSceneUpdate({tfs, items});
        }
    };

    // Machine
    useStore.subscribe(state => 
        [lodash.pickBy(state.programData, (v) => v.dataType === DATA_TYPES.INSTANCE && (v.type === 'machineType')), state.focus, state.collisionsVisible],
        (current, previous) => {
            machineAndTool(current, previous);
        },
        { equalityFn: csArrayEquality }
    );

    // Tools
    useStore.subscribe(state => 
        [lodash.pickBy(state.programData, (v) => v.dataType === DATA_TYPES.INSTANCE && (v.type === 'toolType')), state.focus, state.collisionsVisible],
        (current, previous) => {
            machineAndTool(current, previous);
        },
        { equalityFn: csArrayEquality }
    );

    // Processes
    useStore.subscribe(state => 
        [lodash.pickBy(state.programData, (v) => v.dataType === DATA_TYPES.INSTANCE && (v.type === 'processType')), state.focus],
        (update, prev) => {
            const current = update[0];
            const previous = prev[0]
            const keys = Object.keys(current);
            const state = useStore.getState();
            let tfs = {};
            let items = {};
            let different = false;
            let updateProps = false;
            keys.forEach(key => {
                if (previous[key] && (update[1].includes(key))) {
                    updateProps = true;
                    const entry = current[key];
                    entry.properties.inputs.forEach(input => {
                        const inputObj = state.programData[input];
                        const thing = state.programData[inputObj.properties.thing];
                        tfs[input] = {
                            transformMode: itemTransformMethod(state, input)
                        }
                        items[input] = {
                            hidden: !(state.focus.includes(entry.id) || state.focus.includes(input))
                        };
                        thing.properties.graspPoints.forEach((gp) => {
                            const id = input + gp + '-viz';
                            tfs[id] = {
                                transformMode: itemTransformMethod(state, gp),
                            }
                            items[id] = {
                                hidden: !(state.focus.includes(entry.id) || state.focus.includes(input) || state.focus.includes(gp))
                            }
                        });
                    });
                    
                    entry.properties.outputs.forEach(output => {
                        const outputObj = state.programData[output];
                        const thing = state.programData[outputObj.properties.thing];
                        tfs[output] = {
                            transformMode: itemTransformMethod(state, output)
                        }

                        items[output] = {
                            hidden: !(state.focus.includes(entry.id) || state.focus.includes(output))
                        }

                        thing.properties.graspPoints.forEach((gp) => {
                            const id = output + gp + '-viz';
                            tfs[id] = {
                                transformMode: itemTransformMethod(state, gp)
                            }
                            items[id] = {
                                hidden: !(state.focus.includes(entry.id) || state.focus.includes(output) || state.focus.includes(gp))
                            }
                        });
                    });
                    

                } else if (!previous[key] || (JSON.stringify(current[key]) !== JSON.stringify(previous[key]))) {
                    different = true;
                    const entry = current[key];
                    entry.properties.inputs.forEach(input => {
                        const inputObj = state.programData[input];
                        const thing = state.programData[inputObj.properties.thing];
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
                        });
                    });
                    
                    entry.properties.outputs.forEach(output => {
                        const outputObj = state.programData[output];
                        const thing = state.programData[outputObj.properties.thing];
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
                        });
                    });
                    
                }
            });
            
            if (!updateProps && different) {
                useStore.getState().applySceneUpdate({tfs, items});
            } else if (updateProps) {
                useStore.getState().applyPartialSceneUpdate({tfs, items});
            }
        },
        { equalityFn: csArrayEquality }
    );

    // Human occupancy zones
    useStore.subscribe(state => 
     [lodash.pickBy(state.programData, (v) => v.dataType === DATA_TYPES.INSTANCE && (v.type === 'zoneType' && state.programData[v?.properties?.agent]?.type === 'humanAgentType')), state.collisionsVisible, state.occupancyVisible],
        (update, prev) => {
            const previous = prev[0];
            const current = update[0];
            const keys = Object.keys(current);
            const state = useStore.getState();
            let tfs = {};
            let items = {};
            let different = false;
            let updateProps = false;
            keys.forEach(key => {
                if (previous[key] && (update[1] !== prev[1] || update[2] !== prev[2])) {
                    updateProps = true;
                    const entry = current[key];
                    items[entry.id] = {
                        hidden: !state.occupancyVisible
                    }
                    const collisionObject = state.programData[entry.properties.collision];
            
                    // Now add collisions
                    collisionObject?.properties.componentShapes.forEach((collisionShapeId) => {
                        items[entry.id + collisionShapeId] = {
                            hidden: !(state.collisionsVisible && state.occupancyVisible)
                        }
                    });
                } else if (!previous[key] || (JSON.stringify(current[key]) !== JSON.stringify(previous[key]))) {
                    different = true;
                    const entry = current[key];
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
                }
            });
            
            if (!updateProps && different) {
                useStore.getState().applySceneUpdate({tfs, items});
            } else if (updateProps) {
                useStore.getState().applyPartialSceneUpdate({items});
            }
        },
        { equalityFn: csArrayEquality }
    );

    // Move Trajectory lines and hulls
    // BUG: TODO: when changing the focus (from one issue to another), the lines/hulls still render
    useStore.subscribe(state => 
        [state.issues, state.focus],
        (update, prev) => {
            const state = useStore.getState();
            let lines = {};
            let hulls = {};
            let different = false;
            let updateProps = false;
            let deepestIssue = null;
            for (var i = update[1].length - 1; i >= 0; i--) {
                if (update[0][update[1][i]]) {
                    deepestIssue = update[0][update[1][i]]
                    i = -1;
                }
            }
            let previousDeepestIssue = null;
            for (var i = prev[1].length - 1; i >= 0; i--) {
                if (prev[0][prev[1][i]]) {
                    previousDeepestIssue = prev[0][prev[1][i]]
                    i = -1;
                }
            }
            
            if (!shallow(deepestIssue, previousDeepestIssue)) {
                if (deepestIssue &&
                    state.programData[deepestIssue.focus[0]] &&
                    deepestIssue.sceneData &&
                    deepestIssue.sceneData.vertices) {
                    
                    different = true;
                    let vertKeys = Object.keys(deepestIssue.sceneData.vertices);
                    for (let i = 0; i < vertKeys.length; i++) {
                        lines[deepestIssue.focus[0].concat(vertKeys[i])] = { 
                            name: vertKeys[i],
                            vertices: deepestIssue.sceneData.vertices[vertKeys[i]],
                            frame: 'world',
                            hidden: !update[1].includes(deepestIssue.focus[0]),
                            width: 4
                        };
                    }   
                } else if (previousDeepestIssue && 
                    !deepestIssue &&
                    state.programData[previousDeepestIssue.focus[0]] &&
                    previousDeepestIssue.sceneData &&
                    previousDeepestIssue.sceneData.vertices) {

                    updateProps = true;
                    let vertKeys = Object.keys(previousDeepestIssue.sceneData.vertices);
                    for (let i = 0; i < vertKeys.length; i++) {
                        lines[previousDeepestIssue.focus[0].concat(vertKeys[i])] = { 
                            hidden: !update[1].includes(previousDeepestIssue.focus[0])
                        };
                    } 
                }
            }
        
            if (!shallow(deepestIssue, previousDeepestIssue)) {
                if (deepestIssue &&
                    state.programData[deepestIssue.focus[0]] &&
                    deepestIssue.sceneData &&
                    deepestIssue.sceneData.hulls) {

                    different = true;
                    let vertKeys = Object.keys(deepestIssue.sceneData.hulls);
                    for (let i = 0; i < vertKeys.length; i++) {
                        hulls[deepestIssue.focus[0].concat(vertKeys[i])] = {
                            name: vertKeys[i],
                            vertices: deepestIssue.sceneData.hulls[vertKeys[i]].vertices,
                            color: deepestIssue.sceneData.hulls[vertKeys[i]].color,
                            frame: 'base_link',
                            hidden: !update[1].includes(deepestIssue.focus[0]),
                            width: 2
                        };
                    }   
                } else if (previousDeepestIssue && 
                    !deepestIssue &&
                    state.programData[previousDeepestIssue.focus[0]] &&
                    previousDeepestIssue.sceneData &&
                    previousDeepestIssue.sceneData.hulls) {

                    updateProps = true;
                    let vertKeys = Object.keys(previousDeepestIssue.sceneData.hulls);
                    for (let i = 0; i < vertKeys.length; i++) {
                        hulls[previousDeepestIssue.focus[0].concat(vertKeys[i])] = { 
                            hidden: !update[1].includes(previousDeepestIssue.focus[0])
                        };
                    } 
                }
            }
            
            if (!updateProps && different) {
                useStore.getState().applySceneUpdate({lines, hulls});
            } else if (updateProps) {
                useStore.getState().applyPartialSceneUpdate({lines, hulls});
            }
        },
        { equalityFn: csArrayEquality }
    );

    const locationWaypoint = (update, prev) => {

        const current = update[0];
        const previous = prev[0];

        const keys = Object.keys(current);
        const state = useStore.getState();
        let tfs = {};
        let items = {};
        let different = false;
        let updateProps = false;

        let focusedTrajectoryChildren = [];
        state.focus.forEach((entry) => {
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
        let prevfocusedTrajectoryChildren = [];
        prev[1].forEach((entry) => {
            if (state.programData[entry]?.type === "moveTrajectoryType" || state.programData[entry]?.type === "trajectoryType") {
                let trajectoryTmp = null;

                if (state.programData[entry]?.type === "moveTrajectoryType") {
                    trajectoryTmp = state.programData[state.programData[entry].properties.trajectory];
                } else {
                    trajectoryTmp = state.programData[entry]
                }

                let trajectory = trajectoryTmp?.ref ? state.programData[trajectoryTmp.ref] : trajectoryTmp;

                if (trajectory && state.programData[trajectory.properties.startLocation]?.ref && state.programData[trajectory.properties.endLocation]?.ref) {
                    prevfocusedTrajectoryChildren.push(state.programData[trajectory.properties.startLocation].ref);
                    trajectory.properties.waypoints.forEach((wp) => {
                        prevfocusedTrajectoryChildren.push(state.programData[wp].ref);
                    });
                    prevfocusedTrajectoryChildren.push(state.programData[trajectory.properties.endLocation].ref);
                }
            }
        });

        let robotAgent = null;
        let gripperAgent = null;
        Object.values(state.programData).forEach(entry => {
            if (entry.type === 'robotAgentType') {
                robotAgent = entry;
            } else if (entry.type === 'gripperType') {
                gripperAgent = entry;
            }
        })

        keys.forEach(key => {
            if (previous[key] && (focusedTrajectoryChildren.includes[key] !== prevfocusedTrajectoryChildren.includes(key))) {
                updateProps = true;
                const entry = current[key];
                const focused = state.focus.includes(entry.id);
                const trajectoryFocused = focusedTrajectoryChildren.includes(entry.id);
                
                const updateItem = (link) => {
                    let id = 'ghost-' + entry.id + link;
                    let linkData = state.programData[link];
                    let meshObject = state.programData[linkData.properties.mesh];

                    if (meshObject) {
                        items[id] = {
                            hidden: !(update[2] && trajectoryFocused),
                        }
                    }
                }

                // Add ghost robot to location
                Object.keys(entry.properties?.states[robotAgent.id][gripperAgent.id].links).forEach(link => {
                    updateItem(link);
                });

                // Add ghost gripper to location
                Object.keys(gripperAgent.properties.gripperFrames).forEach(link => {
                    updateItem(link);
                })
                
                
                poseDataToShapes(entry, state.frame, state.programData).forEach((shape) => {
                    const transform = state.focus.includes('translate')
                        ? 'translate'
                        : state.focus.includes('rotate')
                            ? 'rotate'
                            : 'inactive'
                    items[shape.uuid] = {
                        highlighted: focused,
                        hidden: !focused && !trajectoryFocused,
                        transformMode: shape.uuid.includes('pointer') && focused ? transform : "inactive"
                    };
                });
            } else if (!previous[key] || (JSON.stringify(current[key]) !== JSON.stringify(previous[key]))) {
                different = true;
                const entry = current[key];
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
                            hidden: !(update[2] && trajectoryFocused),
                        }
                    }
                }

                // Add ghost robot to location
                Object.keys(entry.properties?.states[robotAgent.id][gripperAgent.id].links).forEach(link => {
                    addItem(link, false);
                });

                // Add ghost gripper to location
                Object.keys(gripperAgent.properties.gripperFrames).forEach(link => {
                    addItem(link, true);
                })

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
        });
        
        if (!updateProps && different) {
            useStore.getState().applySceneUpdate({tfs, items});
        } else if (updateProps) {
            useStore.getState().applyPartialSceneUpdate({tfs, items});
        }
    }

    // Locations
    useStore.subscribe(state => 
        [lodash.pickBy(state.programData, (v) => v.dataType === DATA_TYPES.INSTANCE && (v.type === 'locationType')), state.focus, state.robotPreviewVisible],
        (current, previous) => {
            locationWaypoint(current, previous);
        },
        { equalityFn: csArrayEquality }
    );

    // Waypoints
    useStore.subscribe(state => 
        [lodash.pickBy(state.programData, (v) => v.dataType === DATA_TYPES.INSTANCE && (v.type === 'waypointType')), state.focus, state.robotPreviewVisible],
        (current, previous) => {
            locationWaypoint(current, previous);
        },
        { equalityFn: csArrayEquality }
    );

    // Trajectories
    useStore.subscribe(state => 
        [lodash.pickBy(state.programData, (v) => v.dataType === DATA_TYPES.INSTANCE && (v.type === 'trajectoryType')), state.focus, state.frame],
        (update, prev) => {
            const current = update[0];
            const previous = prev[0];
    
            const keys = Object.keys(current);
            const state = useStore.getState();
            let lines = {};
            let different = false;
            let updateProps = false;

            let visualizeIssue = false;
            for (var i = update[1].length - 1; i >= 0; i--) {
                if (state.issues[update[1][i]]) {
                    visualizeIssue = true;
                    i = -1;
                }
            }

            keys.forEach(key => {
                const entry = current[key];
                let moveTrajectoryId = null;
                update[1].forEach(focusItem => {
                    let obj = state.programData[focusItem];
                    if (obj?.type === "moveTrajectoryType" && obj?.properties?.trajectory === key) {
                        moveTrajectoryId = obj.id;
                    }
                });
                let prevMoveTrajectoryId = null;
                prev[1].forEach(focusItem => {
                    let obj = state.programData[focusItem];
                    if (obj?.type === "moveTrajectoryType" && obj?.properties?.trajectory === key) {
                        prevMoveTrajectoryId = obj.id;
                    }
                });
                
                // Trajectory is shown || move trajectory is shown || prev move tracjectory is no longer shown
                if ((update[1].includes(key) !== prev[1].includes(key)) ||
                    (update[1].includes(moveTrajectoryId) !== prev[1].includes(moveTrajectoryId)) ||
                    (update[1].includes(prevMoveTrajectoryId) !== prev[1].includes(prevMoveTrajectoryId))) {

                    updateProps = true;

                    const hidden = visualizeIssue || !(update[1].includes(key) || update[1].includes(moveTrajectoryId));
            
                    let poses = []
                    if (entry.properties.startLocation) {
                        poses.push(state.programData[entry.properties.startLocation]);
                    }
                    entry.properties.waypoints.forEach(waypointId => {
                        poses.push(state.programData[waypointId])
                    })
                    if (entry.properties.endLocation) {
                        poses.push(state.programData[entry.properties.endLocation]);
                    }

                    const vertices = poses.map(pose => {
                        let pos = pose.ref ? state.programData[pose.ref].properties.position : pose.properties.position;
                        let reachable = pose.ref ? state.programData[pose.ref].properties.reachable : pose.properties.reachable;
                        let color = { ...DEFAULT_TRAJECTORY_COLOR };
                        if (update[2] === 'performance' && !reachable) {//pose, frame, focused, locationOrWaypoint
                            color = { ...UNREACHABLE_COLOR };
                        } else if (update[2] === 'safety' && occupancyOverlap(pos, state.programData)) {
                            color = { ...OCCUPANCY_ERROR_COLOR };
                        }
                        return {
                            position: pos,
                            color
                        }
                    });

                    lines[key] = {
                        vertices: vertices,
                        hidden: hidden
                    }

                } else if (!previous[key] || (JSON.stringify(current[key]) !== JSON.stringify(previous[key]))) {
                    different = true;
            
                    const hidden = visualizeIssue || (!state.focus.includes(entry.id) && !state.focus.includes(moveTrajectoryId));
            
                    let poses = []
                    if (entry.properties.startLocation) {
                        poses.push(state.programData[entry.properties.startLocation]);
                    }
                    entry.properties.waypoints.forEach(waypointId => {
                        poses.push(state.programData[waypointId])
                    })
                    if (entry.properties.endLocation) {
                        poses.push(state.programData[entry.properties.endLocation]);
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
            
                    // let program = lodash.filter(state.programData, function (v) { return v.type === 'programType' && v.dataType === DATA_TYPES.INSTANCE })[0];
                    // let steps = program.properties.compiled["{}"]?.steps;
                    // let sceneTmp = (steps && moveTrajectoryId) ? steps.filter(step => step.type === STEP_TYPE.SCENE_UPDATE && step.source === moveTrajectoryId) : [];
                    // let programModel = createEnvironmentModel(state.programData);
                    // let gripOffsetID = gripperAgent.id + '-gripOffset';
                    // let eePoseVerts = sceneTmp.map(sceneUpdate => {
                    //     Object.keys(sceneUpdate.data.links).forEach(link => {
                    //         programModel = updateEnvironModel(programModel, link, sceneUpdate.data.links[link].position, sceneUpdate.data.links[link].rotation);
                    //     });
                    //     let { position, rotation } = queryWorldPose(programModel, gripOffsetID, '');
                    //     return {
                    //         position: position,
                    //         color: { ...DEFAULT_TRAJECTORY_COLOR }
                    //     }
                    // });
                    // lines[entry.id.concat('-eePose')] = { name: entry.name.concat('-eePose'), vertices: eePoseVerts, frame: 'world', hidden, width: 2 };
            
                    lines[entry.id] = { name: entry.name, vertices, frame: 'world', hidden, width: 2 }
                }
            });
            
            if (!updateProps && different) {
                useStore.getState().applySceneUpdate({lines});
            } else if (updateProps) {
                useStore.getState().applyPartialSceneUpdate({lines});
            }
        },
        { equalityFn: csArrayEquality }
    );
}

// Subscribes to the compiled store
export const computedSliceCompiledSubscribe = (useStore) => {

}



export const computedSlice = (state) => {
    let executablePrimitives = {};

    // ===================== TFs =====================
    let tfs = {};
    let items = {};
    let texts = {};
    let hulls = {};
    let lines = {};

    let reversedFocus = [];
    for (var i = state.focus.length - 1; i >= 0; i--) {
        reversedFocus.push(state.focus[i]);
    }

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

    // const robotAgent = Object.values(state.programData).filter(v => v.type === "robotAgentType")[0];
    // const gripperAgent = Object.values(state.programData).filter(v => v.type === "gripperType")[0];
    let robotAgent = null;
    let gripperAgent = null;
    Object.values(state.programData).forEach(entry => {
        if (entry.type === 'robotAgentType') {
            robotAgent = entry;
        } else if (entry.type === 'gripperType') {
            gripperAgent = entry;
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


    // Pinch Point visualizations
    if (deepestIssue && deepestIssue.code === 'pinchPoints' && state.programData[deepestIssue.focus[0]]) {
        const pinchPointAnimations = pinchpointAnimationFromExecutable(robotAgent, state.programData[deepestIssue.focus[0]].properties.compiled["{}"].steps)
        Object.keys(pinchPointAnimations).forEach(field => {
            items[field] = {
                shape: 'sphere',
                rotation: { w: 1, x: 0, y: 0, z: 0 },
                ...pinchPointAnimations[field]
            }
        });
    }

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

    // stepsToAnimation(state, tfs, items);

    return ({
        executablePrimitives,
        tfs,
        items,
        lines,
        hulls,
        texts
    })
}
