// import { objectMap } from "./helpers";
import { 
    poseToColor, 
    poseDataToShapes, 
    trajectoryDataToLine 
} from './helpers';
import { INITIAL_SIM, COLLISION_MESHES } from './initialSim';

const ROBOT_PARTS = Object.keys(INITIAL_SIM.staticScene).filter(v => v.includes('robot'));
const GRIPPER_PARTS = Object.keys(INITIAL_SIM.staticScene).filter(v => v.includes('gripper'));

export const ComputedSlice = {

    computed: {
        items: function () {
            let items = {};
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
                    transformMode: 'inactive',
                    highlighted,
                    onClick: (e) => {
                        console.log('clicked '+item.name)
                        e.stopPropagation();
                        this.setFocusItem('scene', itemKey);
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
                        transformMode: 'inactive',
                        highlighted: false,
                        wireframe: true,
                        hidden: !this.collisionsVisible,
                        onClick: (e) => { },
                        onMove: (transform) => { console.log(transform) }
                    };
                }
            })

            // TODO: Handle whether 
            Object.keys(this.data.locations).forEach(location_uuid => {
                const item = this.data.locations[location_uuid]
                const focused = this.focusItem.uuid === location_uuid || this.secondaryFocusItem.uuid === location_uuid;
                const trajectoryFocused = Object.values(this.data.trajectories).some(trajectory => trajectory.start_location_uuid === location_uuid || trajectory.end_location_uuid === location_uuid);
                // Handle in the case where the trajectory is focused
                const color = poseToColor(item, this.frame, focused || trajectoryFocused);
                poseDataToShapes(item, this.frame).forEach(shape => {
                    items[shape.uuid] = { ...shape, highlighted: focused, hidden: !focused && !trajectoryFocused, color };
                })
            })

            Object.keys(this.data.waypoints).forEach(waypoint_uuid => {
                const item = this.data.waypoints[waypoint_uuid];
                const focused = this.focusItem.uuid === waypoint_uuid || this.secondaryFocusItem.uuid === waypoint_uuid;
                const trajectoryFocused = Object.values(this.data.trajectories).some(trajectory => trajectory.waypoint_uuids.some(trajectory_waypoint => trajectory_waypoint === waypoint_uuid));
                // Handle in the case where the trajectory is focused
                const color = poseToColor(item, this.frame, focused || trajectoryFocused);
                poseDataToShapes(item, this.frame).forEach(shape => {
                    items[shape.uuid] = { ...shape, highlighted: focused, hidden: !focused && !trajectoryFocused, color };
                })
            })
            console.log(items)
            return items
        },
        lines: function () {
            const lines = {};
            Object.values(this.data.trajectories).forEach(trajectory=>{
                const hidden = this.focusItem.uuid !== trajectory.uuid && this.secondaryFocusItem.uuid !== trajectory.uuid
                lines[trajectory.uuid] = {...trajectoryDataToLine(trajectory,this.data.locations,this.data.waypoints,this.frame),hidden,width:2}
            })
            return lines
        },
        hulls: function() {

            return {}
        },
        tfs: function() {
            return INITIAL_SIM.tfs
        }
    },
}