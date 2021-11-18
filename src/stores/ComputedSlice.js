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
import { INITIAL_SIM, COLLISION_MESHES, EVD_MESH_LOOKUP } from './initialSim';

const ROBOT_PARTS = Object.keys(INITIAL_SIM.staticScene).filter(v => v.includes('robot'));
const GRIPPER_PARTS = Object.keys(INITIAL_SIM.staticScene).filter(v => v.includes('gripper'));


export const ComputedSlice = {
    computed: {
        executablePrimitives: function () {
            // let executableLookup = {};
            // executableLookup[this.uuid] = executablePrimitive(this.uuid,this);
            // this.primitiveIds.forEach(primitiveId=>{
            //     executableLookup[primitiveId] = executablePrimitive(primitiveId,this)
            // })
            // return executableLookup
            return executablePrimitives(this)
        },
        tfs: function () {
            let tfs = { ...INITIAL_SIM.tfs };
            Object.values(this.data).filter(v => v.type === 'thing').forEach(thing => {
                // for now, place the things at origin. 
                // console.log(thing)
                tfs[thing.uuid] = {
                    frame: 'world',
                    translation: { x: 0, y: 0, z: 0 },
                    rotation: { w: 1, x: 0, y: 0, z: 0 }
                }
            })
            // Object.values(this.data.regions).forEach(region=>{
            //     tfs[region.uuid] = {
            //         frame:region.link,
            //         translation:region.center_position,
            //         rotation:region.center_orientation
            //     }
            // })

            if (this.focusItem.uuid) {
                const executable = this.executablePrimitives[this.focusItem.uuid];
                if (executable) {
                    tfs = tfAnimationFromExecutable(executable, tfs)
                } //else if (this.data[this.focusItem.uuid].frames) {
                //  console.log(this.data[this.focusItem.uuid].frames)
                //    tfs = { ...tfs, ...robotFramesFromPose(this.data[this.focusItem.uuid]) }
                //}
            }
            return tfs
        },
        items: function () {
            let items = {};
            const focusedTrajectoryChildren = this.focusItem.type === 'trajectory' ? [
                this.data[this.focusItem.uuid].start_location_uuid,
                ...this.data[this.focusItem.uuid].waypoint_uuids,
                this.data[this.focusItem.uuid].end_location_uuid,
            ] : []

            // Add items from the initial static scene
            Object.keys(INITIAL_SIM.staticScene).forEach(itemKey => {
                const item = INITIAL_SIM.staticScene[itemKey];
                let highlighted = false;
                if (ROBOT_PARTS.indexOf(this.focusItem.uuid) >= 0 && ROBOT_PARTS.indexOf(itemKey) >= 0) {
                    highlighted = true
                } else if (GRIPPER_PARTS.indexOf(this.focusItem.uuid) >= 0 && GRIPPER_PARTS.indexOf(itemKey) >= 0) {
                    highlighted = true
                } else if (this.focusItem.uuid === itemKey || this.secondaryFocusItem.uuid === itemKey) {
                    highlighted = true
                }
                items[itemKey] = {
                    shape: item.shape,
                    name: item.name,
                    frame: item.frame,
                    position: item.position,
                    rotation: item.rotation,
                    color: item.color,
                    scale: item.scale,
                    transformMode: "inactive",
                    highlighted,
                    onClick: (e) => {
                        if (
                            this.focusItem.transformMode === "translate" ||
                            this.focusItem.transformMode === "rotate" ||
                            this.secondaryFocusItem.transformMode === "translate" ||
                            this.secondaryFocusItem.transformMode === "rotate"
                        ) {

                        } else {
                            console.log('clicked ' + item.name)
                            e.stopPropagation();
                            this.setFocusItem('scene', itemKey);
                        }

                    },
                    onMove: (transform) => { console.log(transform) }
                }
                if (COLLISION_MESHES[item.shape]) {
                    items[itemKey + '-collision'] = {
                        shape: COLLISION_MESHES[item.shape],
                        name: item.name + ' Collision',
                        frame: item.frame,
                        position: item.position,
                        rotation: item.rotation,
                        scale: item.scale,
                        color: { r: 250, g: 0, b: 0, a: 0.6 },
                        transformMode: "inactive",
                        highlighted: false,
                        wireframe: true,
                        hidden: !this.collisionsVisible,
                        onClick: (_) => { },
                        onMove: (transform) => { console.log(transform) }
                    };
                }
            })

            // Add occupancy zones
            Object.values(this.data).forEach(entry => {
                if (entry.type === 'zone' && this.data[entry.agent].type === 'human-agent') {
                    items[entry.uuid] = {
                        shape: 'cube',
                        name: entry.name,
                        frame: 'world',
                        position: entry.position,
                        rotation: entry.orientation,
                        color: { ...OCCUPANCY_ERROR_COLOR, a: 0.2 },
                        scale: entry.scale,
                        transformMode: "inactive",
                        highlighted: false,
                        onClick: (_) => { },
                        hidden: !this.occupancyVisible
                    }
                } else if (entry.type === 'machine') {
                    // console.log(entry.name)
                    // const mesh = EVD_MESH_LOOKUP[entry.mesh]
                    items[entry.uuid] = {
                        shape: entry.mesh,
                        name: entry.name,
                        frame: entry.link,
                        position: entry.position,
                        rotation: entry.orientation,
                        scale: entry.link === 'assembly_jig_link' ? { x: 0.2, y: 0.2, z: 0.2 } : { x: 1, y: 1, z: 1 },
                        transformMode: 'inactive',
                        highlighted: this.focusItem.uuid === entry.uuid,
                        onClick: (e) => {
                            if (this.focusItem.transformMode === "translate" || this.focusItem.transformMode === "rotate") {

                            } else {
                                console.log('clicked ' + entry.name)
                                e.stopPropagation();
                                this.setFocusItem('data', entry.uuid);
                            }

                        },
                    }
                    // Now add collisions
                    items[entry.uuid + '-collision'] = {
                        shape: entry.collisionMesh,
                        name: entry.name + ' Collision',
                        frame: entry.link,
                        position: entry.position,
                        rotation: entry.orientation,
                        scale: entry.link === 'assembly_jig_link' ? { x: 0.2, y: 0.2, z: 0.2 } : { x: 1, y: 1, z: 1 },
                        transformMode: 'inactive',
                        highlighted: false,
                        color: { r: 250, g: 0, b: 0, a: 0.6 },
                        wireframe: true,
                        hidden: !this.collisionsVisible,
                        onClick: (_) => { },
                    }

                    // Now enumerate the inputs and render the zones and items.
                    // if (this.focusItem.uuid === machine.uuid) {
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

                    //items = { ...items, ...machineDataToPlaceholderPreviews(machine, this.data.thingTypes, this.data.regions, this.data.placeholders) }
                } else if (entry.type === 'location' || entry.type === 'waypoint') {
                    const focused = this.focusItem.uuid === entry.uuid || this.secondaryFocusItem.uuid === entry.uuid;
                    const trajectoryFocused = focusedTrajectoryChildren.includes(entry.uuid);

                    // Handle in the case where the trajectory is focused
                    let color = entry.type === 'location' ? { ...DEFAULT_LOCATION_COLOR } : { ...DEFAULT_WAYPOINT_COLOR };
                    //console.log(item.joints.reachable);
                    if (this.frame === 'performance' && !entry.reachable) {//pose, frame, focused, locationOrWaypoint
                        //console.log("entered");
                        color = { ...UNREACHABLE_COLOR };
                    } else if (this.frame === 'safety' && occupancyOverlap(entry.position, this.data)) {
                        color = { ...OCCUPANCY_ERROR_COLOR };
                    }
                    if (trajectoryFocused) {
                        const idx = focusedTrajectoryChildren.indexOf(entry.uuid);
                        color.a = (time) => 0.5 * Math.pow(Math.E, -Math.sin(time / 800 + idx * 0.98));
                    }

                    const debouncedOnMove = debounce(transform => {
                        this.setPoseTransform(entry.uuid, transform);
                        this.requestJointProcessorUpdate('location', entry.uuid)
                    }, 1000)

                    poseDataToShapes(entry, this.frame, this.data).forEach((shape) => {
                        items[shape.uuid] = {
                            ...shape,
                            highlighted: focused,
                            hidden: !focused && !trajectoryFocused,
                            color,
                            transformMode: shape.uuid.includes('pointer') && focused ? this.focusItem.transformMode : "inactive",
                            onMove: shape.uuid.includes('pointer') && focused && this.focusItem.transformMode !== 'inactive' ? debouncedOnMove : (_) => { }
                        };
                        // e => setItemProperty('location', location_uuid, 'position', { ...this.data.locations[location_uuid].position, x: e[0], y: e[1], z: e[2] }); 


                    })
                }
            })

            // Pinch Point visualizations
            if (this.issues[this.secondaryFocusItem.uuid] && this.executablePrimitives[this.focusItem.uuid] && this.issues[this.secondaryFocusItem.uuid].code === 'pinchPoints') {
                console.log('---------show pinchpoints---------')
                const pinchPointAnimations = pinchpointAnimationFromExecutable(this.executablePrimitives[this.focusItem.uuid])
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
            // Object.values(this.data.regions).forEach(region=>{
            //     const debouncedOnMove = debounce(transform=>{
            //         this.setRegionTransform(region.uuid,transform);
            //     },1000)
            //     items[region.uuid] = {
            //         shape: region.uncertainty_x ? 'cube' : 'sphere',
            //         frame: region.uuid,
            //         position: {x:0,y:0,z:0},
            //         rotation: {w:1,x:0,y:0,z:0},
            //         scale: region.uncertainty_x ? {x:region.uncertainty_x*5,y:region.uncertainty_y*5,z:region.uncertainty_z*5} : {x:region.uncertainty_radius*5,y:region.uncertainty_radius*5,z:region.uncertainty_radius*5},
            //         transformMode: this.secondaryFocusItem.uuid === region.uuid ? this.secondaryFocusItem.transformMode : 'inactive',
            //         color:{r:0,g:0,b:0,a:0.3},
            //         highlighted: this.secondaryFocusItem.uuid === region.uuid,
            //         hidden: true,
            //         onClick: (_)=>{},
            //         onMove: debouncedOnMove
            //     }
            // })

            // if (this.focusItem.type === 'machine') {
            //     Object.values(this.data.machines[this.focusItem.uuid].inputs).forEach(regionInfo=>{
            //         regionInfo.forEach(input=>focusedInputs.push(input.region_uuid))
            //     });
            //     Object.values(this.data.machines[this.focusItem.uuid].outputs).forEach(regionInfo=>{
            //         regionInfo.forEach(output=>focusedOutputs.push(output.region_uuid))
            //     });
            // }

            

            


            return items
        },
        lines: function () {

            const lines = {};

            Object.values(this.executablePrimitives).forEach(ePrim => {
                if (ePrim) {
                    Object.values(ePrim).forEach(primitive => {
                        if (primitive.type === "move-trajectory") {
                            const hidden = this.focusItem.uuid !== primitive.uuid && this.secondaryFocusItem.uuid !== primitive.uuid;
                            if (this.secondaryFocusItem.type === "issue") {
                                const currentIssue = this.issues[this.secondaryFocusItem.uuid];
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

            Object.values(this.data).filter(v=>v.type==='trajectory').forEach(trajectory => {
                const hidden = this.focusItem.uuid !== trajectory.uuid && this.secondaryFocusItem.uuid !== trajectory.uuid;
                let poses = []
                if (trajectory.startLocation) {
                    poses.push(this.data.locations[trajectory.startLocation])
                }
                trajectory.waypoints.forEach(waypointId => {
                    poses.push(this.data[waypointId])
                })
                if (trajectory.endLocation) {
                    poses.push(this.data.locations[trajectory.endLocation])
                }
                const vertices = poses.map(pose => {
                    let color = { ...DEFAULT_TRAJECTORY_COLOR };
                    //console.log(item.joints.reachable);
                    if (this.frame === 'performance' && !pose.joints.reachable) {//pose, frame, focused, locationOrWaypoint
                        //console.log("entered");
                        color = { ...UNREACHABLE_COLOR };
                    } else if (this.frame === 'safety' && occupancyOverlap(pose.position, this.data.occupancyZones)) {
                        color = { ...OCCUPANCY_ERROR_COLOR };
                    }
                    return {
                        position: pose.position,
                        color
                    }
                })
                lines[trajectory.uuid] = { name: trajectory.name, vertices, frame: 'world', hidden, width: 2 }
            })
            return lines
        },
        hulls: function () {

            let hulls = {}

            Object.values(this.executablePrimitives).forEach(ePrim => {
                if (ePrim) {
                    Object.values(ePrim).forEach(primitive => {
                        if (primitive.type === "node.primitive.move-trajectory.") {
                            const hidden = this.focusItem.uuid !== primitive.uuid && this.secondaryFocusItem.uuid !== primitive.uuid;
                            if (this.secondaryFocusItem.type === "issue") {
                                const currentIssue = this.issues[this.secondaryFocusItem.uuid];
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

            return hulls
        },
    },
}