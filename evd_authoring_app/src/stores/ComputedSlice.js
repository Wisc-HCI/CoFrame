// import { objectMap } from "./helpers";
import { 
    poseDataToShapes, 
    DEFAULT_WAYPOINT_COLOR,
    DEFAULT_LOCATION_COLOR,
    UNREACHABLE_COLOR,
    OCCUPANCY_ERROR_COLOR,
    occupancyOverlap,
    DEFAULT_TRAJECTORY_COLOR,
    executablePrimitive,
    durationEstimate,
    idleTimeEstimate,
    tfAnimationFromExecutable,
    robotFramesFromPose
} from './helpers';
import debounce from 'lodash.debounce';
// import throttle from 'lodash.throttle';
import { INITIAL_SIM, COLLISION_MESHES, EVD_MESH_LOOKUP } from './initialSim';

const ROBOT_PARTS = Object.keys(INITIAL_SIM.staticScene).filter(v => v.includes('robot'));
const GRIPPER_PARTS = Object.keys(INITIAL_SIM.staticScene).filter(v => v.includes('gripper'));


export const ComputedSlice = {
    computed: {
        executablePrimitives: function() {
            let executableLookup = {};
            executableLookup[this.uuid] = executablePrimitive(this.uuid,this);
            this.primitiveIds.forEach(primitiveId=>{
                executableLookup[primitiveId] = executablePrimitive(primitiveId,this)
            })
            return executableLookup
        },
        tfs: function() {
            let tfs = {...INITIAL_SIM.tfs};
            Object.values(this.data.placeholders).forEach(placeholder=>{
                // for now, place the placeholders at origin. 
                // console.log(placeholder)
                tfs[placeholder.uuid] = {
                    frame:'world',
                    translation:{x:0,y:0,z:0},
                    rotation:{w:1,x:0,y:0,z:0}
                }
            })
            if (this.focusItem.uuid) {
                const executable = this.executablePrimitives[this.focusItem.uuid];
                console.log(executable);
                console.log("this is duration : " + durationEstimate(executable));
                console.log("this is delay: " + idleTimeEstimate(executable));
                if (executable) {
                    tfs = tfAnimationFromExecutable(executable,tfs)
                } else if (this.focusItem.type === 'location' && this.data.locations[this.focusItem.uuid].frames) {
                    console.log(this.data.locations[this.focusItem.uuid].frames)
                    tfs = {...tfs,...robotFramesFromPose(this.data.locations[this.focusItem.uuid])}
                } else if (this.focusItem.type === 'waypoint' && this.data.waypoints[this.focusItem.uuid].frames) {
                    tfs = {...tfs,...robotFramesFromPose(this.data.waypoints[this.focusItem.uuid])}
                }
            }
            return tfs
        },
        items: function () {
            let items = {};
            const focusedTrajectoryChildren = this.focusItem.type === 'trajectory' ? [
                this.data.trajectories[this.focusItem.uuid].start_location_uuid,
                ...this.data.trajectories[this.focusItem.uuid].waypoint_uuids,
                this.data.trajectories[this.focusItem.uuid].end_location_uuid,
            ] : []

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
                        if (this.focusItem.transformMode === "translate" || this.focusItem.transformMode ===  "rotate"){
                            
                        }else{
                            console.log('clicked '+item.name)
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

            Object.values(this.data.occupancyZones).forEach(zone=>{
                if (zone.occupancy_type === 'human') {
                    items[zone.uuid] = {
                        shape: 'cube',
                        name: zone.name,
                        frame: 'world',
                        position: {x:zone.position_x,y:zone.position_z,z:0.25},
                        rotation: {w:1,x:0,y:0,z:0},
                        color: {...OCCUPANCY_ERROR_COLOR,a:0.2},
                        scale: {x:zone.scale_x,y:zone.scale_z,z:2},
                        transformMode: "inactive",
                        highlighted: false,
                        onClick: (_) => {},
                        hidden: !this.occupancyVisible
                    }
                }
                
            })

            let focusedInputs = [];
            let focusedOutputs = [];
            if (this.focusItem.type === 'machine') {
                Object.values(this.data.machines[this.focusItem.uuid].inputs).forEach(regionInfo=>{
                    regionInfo.forEach(input=>focusedInputs.push(input.region_uuid))
                });
                Object.values(this.data.machines[this.focusItem.uuid].outputs).forEach(regionInfo=>{
                    regionInfo.forEach(output=>focusedOutputs.push(output.region_uuid))
                });
            }

            Object.values(this.data.machines).forEach(machine=>{
                const mesh = EVD_MESH_LOOKUP[machine.mesh_id]
                items[machine.uuid] = {
                    shape:mesh,
                    name:machine.name,
                    frame: machine.pose_offset.link,
                    position: machine.pose_offset.position,
                    rotation: machine.pose_offset.orientation,
                    scale: machine.pose_offset.link === 'assembly_jig_link' ? { x: 0.2, y: 0.2, z: 0.2 } : {x: 1, y: 1, z: 1},
                    transformMode: 'inactive',
                    highlighted: this.focusItem.uuid === machine.uuid,
                    onClick: (e) => {
                        if (this.focusItem.transformMode === "translate" || this.focusItem.transformMode ===  "rotate"){
                            
                        }else{
                            console.log('clicked '+machine.name)
                            e.stopPropagation();
                            this.setFocusItem('machine', machine.uuid);
                        }
                        
                    },
                }
                // Now add collisions
                items[machine.uuid+'-collision'] = {
                    shape:COLLISION_MESHES[mesh],
                    name: machine.name + ' Collision',
                    frame: machine.pose_offset.link,
                    position: machine.pose_offset.position,
                    rotation: machine.pose_offset.orientation,
                    scale: machine.pose_offset.link === 'assembly_jig_link' ? { x: 0.2, y: 0.2, z: 0.2 } : {x: 1, y: 1, z: 1},
                    transformMode: 'inactive',
                    highlighted: false,
                    color: { r: 250, g: 0, b: 0, a: 0.6 },
                    wireframe: true,
                    hidden: !this.collisionsVisible,
                    onClick: (_) => {},
                }

                // Now enumerate the inputs and render the zones and items.
                Object.keys(machine.inputs).forEach(thingType=>{
                    machine.inputs[thingType].forEach(zoneInfo=>{
                        const region = this.data.regions[zoneInfo.region_uuid];
                        const debouncedOnMove = debounce(transform=>{
                            this.setRegionTransform(region.uuid,transform);
                        },1000)
                        let color = {r:245,g:206,b:66,a:0.4};
                        const machineInput = focusedInputs.includes(region.uuid);
                        const machineOutput = focusedOutputs.includes(region.uuid);
                        if (machineInput && !machineOutput) {
                            color = {r:245,g:105,b:66,a:0.4}
                        } else if (!machineInput && machineOutput) {
                            color = {r:66,g:245,b:156,a:0.4}
                        }
                        
                        items[zoneInfo.region_uuid] = {
                            shape: region.uncertainty_x ? 'cube' : 'sphere',
                            frame: region.link,
                            position: region.center_position,
                            rotation: region.center_orientation,
                            scale: region.uncertainty_x ? {x:region.uncertainty_x*5,y:region.uncertainty_y*5,z:region.uncertainty_z*5} : {x:region.uncertainty_radius*5,y:region.uncertainty_radius*5,z:region.uncertainty_radius*5},
                            transformMode: this.secondaryFocusItem.uuid === region.uuid ? this.secondaryFocusItem.transformMode : 'inactive',
                            color,
                            highlighted: this.secondaryFocusItem.uuid === region.uuid,
                            hidden: !(this.focusItem.uuid === machine.uuid || this.secondaryFocusItem.uuid === region.uuid),
                            onClick: (_)=>{},
                            onMove: debouncedOnMove
                        }
                        // items[zoneInfo.region_uuid+thingType] = {
                        //     shape: region.uncertainty_x ? 'cube' : 'sphere',
                        //     frame: region.link,
                        //     position: region.center_position,
                        //     rotation: {w:1,x:0,y:0,z:0},
                        //     scale: {x:region.uncertainty_x*5,y:region.uncertainty_y*5,z:region.uncertainty_z*5},
                        //     transformMode: 'inactive',
                        //     color: {r:100,g:200,b:200,a:0.4},
                        //     highlighted: this.secondaryFocusItem.uuid === region.uuid,
                        //     hidden: !(this.focusItem.uuid === machine.uuid || this.secondaryFocusItem.uuid === region.uuid),
                        //     onClick: (_)=>{}
                        // }
                    })
                })

                // Now enumerate the outputs and render the zones and items.
                Object.keys(machine.outputs).forEach(thingType=>{
                    
                    machine.outputs[thingType].forEach(zoneInfo=>{
                        const region = this.data.regions[zoneInfo.region_uuid];
                        const debouncedOnMove = debounce(transform=>{
                            this.setRegionTransform(region.uuid,transform);
                        },1000)
                        items[zoneInfo.region_uuid] = {
                            shape: region.uncertainty_x ? 'cube' : 'sphere',
                            frame: region.link,
                            position: region.center_position,
                            rotation: {w:1,x:0,y:0,z:0},
                            scale: {x:region.uncertainty_x*5,y:region.uncertainty_y*5,z:region.uncertainty_z*5},
                            transformMode: this.secondaryFocusItem.uuid === region.uuid ? this.secondaryFocusItem.transformMode : 'inactive',
                            color: {r:200,g:100,b:100,a:0.4},
                            highlighted: this.secondaryFocusItem.uuid === region.uuid,
                            hidden: !(this.focusItem.uuid === machine.uuid || this.secondaryFocusItem.uuid === region.uuid),
                            onClick: (_)=>{},
                            onMove: debouncedOnMove
                        }
                        const placeholder = this.data.placeholders[zoneInfo.placeholder_uuids[0]];
                        // items[placeholder.uuid] = {

                        // }
                        // console.log(placeholder)
                    })
                })
            })

            Object.keys(this.data.locations).forEach(location_uuid => {
                const item = this.data.locations[location_uuid]
                const focused = this.focusItem.uuid === location_uuid || this.secondaryFocusItem.uuid === location_uuid;
                const trajectoryFocused = focusedTrajectoryChildren.includes(location_uuid);
                
                // Handle in the case where the trajectory is focused
                let color = {...DEFAULT_LOCATION_COLOR};
                //console.log(item.joints.reachable);
                if (this.frame === 'performance' && !item.joints.reachable){//pose, frame, focused, locationOrWaypoint
                    //console.log("entered");
                    color = {...UNREACHABLE_COLOR};
                } else if (this.frame === 'safety' && occupancyOverlap(item.position,this.data.occupancyZones)) {
                    color = {...OCCUPANCY_ERROR_COLOR};
                }
                if (trajectoryFocused) {
                    const idx = focusedTrajectoryChildren.indexOf(location_uuid);
                    color.a = (time)=>0.5*Math.pow(Math.E,-Math.sin(time/800+idx*0.98)); 
                }

                const debouncedOnMove = debounce(transform=>{
                    this.setPoseTransform(location_uuid,transform);
                    this.requestJointProcessorUpdate('location',location_uuid)
                },1000)

                poseDataToShapes(item, this.frame).forEach((shape,i) => {
                    items[shape.uuid] = { 
                        ...shape, 
                        highlighted: focused, 
                        hidden: !focused && !trajectoryFocused, 
                        color,
                        transformMode: shape.uuid.includes('pointer') && focused ? this.focusItem.transformMode : "inactive",
                        onMove: shape.uuid.includes('pointer') && focused && this.focusItem.transformMode !== 'inactive' ? debouncedOnMove : (_)=>{} 
                    }; 
                   // e => setItemProperty('location', location_uuid, 'position', { ...this.data.locations[location_uuid].position, x: e[0], y: e[1], z: e[2] }); 


                })
            })

            Object.keys(this.data.waypoints).forEach(waypoint_uuid => {
                const item = this.data.waypoints[waypoint_uuid];
                const focused = this.focusItem.uuid === waypoint_uuid || this.secondaryFocusItem.uuid === waypoint_uuid;
                const trajectoryFocused = focusedTrajectoryChildren.includes(waypoint_uuid);
                
                // Handle in the case where the trajectory is focused
                let color = {...DEFAULT_WAYPOINT_COLOR};
                //console.log(item.joints.reachable);
                if (this.frame === 'performance' && !item.joints.reachable){//pose, frame, focused, locationOrWaypoint
                    //console.log("entered");
                    color = {...UNREACHABLE_COLOR};
                } else if (this.frame === 'safety' && occupancyOverlap(item.position,this.data.occupancyZones)) {
                    color = {...OCCUPANCY_ERROR_COLOR};
                }
                if (trajectoryFocused) {
                    const idx = focusedTrajectoryChildren.indexOf(waypoint_uuid);
                    color.a = (time)=>0.5*Math.pow(Math.E,-Math.sin(time/800+idx*0.98)); 
                }

                const debouncedOnMove = debounce(transform=>{
                    this.setPoseTransform(waypoint_uuid,transform);
                    this.requestJointProcessorUpdate('waypoint',waypoint_uuid)
                },1000)

                poseDataToShapes(item, this.frame).forEach(shape => {
                   // color.a = (time) => 0.5*Math.pow(Math.E,-Math.sin(time/800+i*0.3));
                    items[shape.uuid] = { 
                        ...shape, 
                        highlighted: focused, 
                        hidden: !focused && !trajectoryFocused, 
                        color,
                        transformMode: shape.uuid.includes('pointer') && focused ? this.focusItem.transformMode : "inactive",
                        onMove: shape.uuid.includes('pointer') && focused && this.focusItem.transformMode !== 'inactive' ? debouncedOnMove : (_)=>{}
                    };
                    
                    
                })
            })
            return items
        },
        lines: function () {
            
            const lines = {};
            Object.values(this.data.trajectories).forEach(trajectory=>{
                const hidden = this.focusItem.uuid !== trajectory.uuid && this.secondaryFocusItem.uuid !== trajectory.uuid;
                let poses = []
                if (trajectory.start_location_uuid) {
                    poses.push(this.data.locations[trajectory.start_location_uuid])
                }
                trajectory.waypoint_uuids.forEach(waypoint_uuid=>{
                    poses.push(this.data.waypoints[waypoint_uuid])
                })
                if (trajectory.end_location_uuid) {
                    poses.push(this.data.locations[trajectory.end_location_uuid])
                }
                const vertices = poses.map(pose=>{
                    let color = {...DEFAULT_TRAJECTORY_COLOR};
                    //console.log(item.joints.reachable);
                    if (this.frame === 'performance' && !pose.joints.reachable){//pose, frame, focused, locationOrWaypoint
                        //console.log("entered");
                        color = {...UNREACHABLE_COLOR};
                    } else if (this.frame === 'safety' && occupancyOverlap(pose.position,this.data.occupancyZones)) {
                        color = {...OCCUPANCY_ERROR_COLOR};
                    }
                    return {
                        position:pose.position,
                        color
                    }
                })
                lines[trajectory.uuid] = {name:trajectory.name,vertices,frame:'world',hidden,width:2}
            })
            return lines
        },
        hulls: function() {
            return {}
        },
    },
}