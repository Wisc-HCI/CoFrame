// import { objectMap } from "./helpers";
import { 
    poseDataToShapes, 
    trajectoryDataToLine,
    DEFAULT_WAYPOINT_COLOR,
    DEFAULT_LOCATION_COLOR,
    UNREACHABLE_COLOR,
    OCCUPANCY_ERROR_COLOR,
    occupancyOverlap,
    DEFAULT_TRAJECTORY_COLOR,
    executablePrimitive
} from './helpers';
import debounce from 'lodash.debounce';
// import throttle from 'lodash.throttle';
import { INITIAL_SIM, COLLISION_MESHES } from './initialSim';

const ROBOT_PARTS = Object.keys(INITIAL_SIM.staticScene).filter(v => v.includes('robot'));
const GRIPPER_PARTS = Object.keys(INITIAL_SIM.staticScene).filter(v => v.includes('gripper'));

export const ComputedSlice = {

    computed: {
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
        tfs: function() {
            return INITIAL_SIM.tfs
        },
        executablePrimitives: function() {
            let executableLookup = {};
            executableLookup[this.uuid] = executablePrimitive(this.uuid,this);
            this.primitiveIds.forEach(primitiveId=>{
                executableLookup[primitiveId] = executablePrimitive(primitiveId,this)
            })
            return executableLookup
        }
    },
}