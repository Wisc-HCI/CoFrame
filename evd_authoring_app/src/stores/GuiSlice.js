import frameStyles from '../frameStyles';
import { INITIAL_SIM, COLLISION_MESHES } from './initialSim';

const ROBOT_PARTS = Object.keys(INITIAL_SIM.staticScene).filter(v => v.includes('robot'));
const GRIPPER_PARTS = Object.keys(INITIAL_SIM.staticScene).filter(v => v.includes('gripper'));

const ACTIVE_TFS = [
  'simulated_base_link',
  'simulated_shoulder_link',
  'simulated_upper_arm_link',
  'simulated_forearm_link',
  'simulated_wrist_1_link',
  'simulated_wrist_2_link',
  'simulated_wrist_3_link'
]

export const GuiSlice = (set, get) => ({
  // EDITOR/SETUP/MAIN
  // The frame specifies the expert (color) frame
  frame: 'safety',
  primaryColor: frameStyles.colors['safety'],
  setFrame: (frame) => set(state => {
    state.frame = frame;
    state.primaryColor = frameStyles.colors[frame];
    if (state.secondaryFocusItem.type === 'issue') {
      state.focusItem = { type: null, uuid: null };
      state.secondaryFocusItem = { type: null, uuid: null };
    }
    // TODO: recolor any items/lines/hulls.
  }),
  // the activeModal specifies the type of modal shown
  activeModal: null,
  setActiveModal: (modal) => set(_ => ({ activeModal: modal })),
  closeModal: () => set(_ => ({ activeModal: null })),
  // the activeDrawer specifies the tab in the SetupEditor that is shown
  activeDrawer: null,
  setActiveDrawer: (drawer) => set(_ => ({ activeDrawer: drawer })),
  // the focusItem specifies the type and uuid of data to focus on
  focusItem: { type: null, uuid: null },
  setFocusItem: (type, uuid) => set(state => {
    // TODO Clear current items.
    state.focusItem = { type: type, uuid: uuid };
    // Clear Highlights
    Object.keys(state.items).forEach(itemKey => state.items[itemKey].highlighted = false);
    Object.keys(state.hulls).forEach(hullKey => state.hulls[hullKey].highlighted = false);
    // Clear any animations that are running
    if (type === 'trajectory') {
      state.lines[state.focusItem.uuid].width = 2;
      const poses = [];
      if (state.data.trajectories[uuid].start_location_uuid) {
        poses.push(state.data.trajectories[uuid].start_location_uuid);
      }
      state.data.trajectories[uuid].waypoint_uuids.forEach((waypoint_uuid,i)=> {
        poses.push(waypoint_uuid);
      })
      if (state.data.trajectories[uuid].end_location_uuid) {
        poses.push(state.data.trajectories[uuid].end_location_uuid); 
      }
      poses.forEach((pose_uuid,i)=>{
        // state.items[pose_uuid+'-tag'].showName = true;
        state.items[pose_uuid+'-tag'].color.a = (time)=>0.3*Math.pow(Math.E,-Math.sin(time/250+i*0.4));
        state.items[pose_uuid+'-pointer'].color.a = (time)=>0.3*Math.pow(Math.E,-Math.sin(time/250+i*0.4));
      })
    } else if (type === 'location' || type === 'waypoint') {
      state.items[uuid+'-tag'].color.a = 1;
      state.items[uuid+'-pointer'].color.a = 1;
      state.items[uuid+'-tag'].highlighted = true;
      state.items[uuid+'-pointer'].highlighted = true;
    }
    if (type === 'scene' && ROBOT_PARTS.indexOf(uuid)>=0) {
      ROBOT_PARTS.forEach(part=>{state.items[part].highlighted = true})
    } else if (type === 'scene' && GRIPPER_PARTS.indexOf(uuid)>=0) {
      GRIPPER_PARTS.forEach(part=>{state.items[part].highlighted = true})
    } else if (type === 'scene') {
      state.items[uuid].highlighted = true
    }

  }),
  clearFocusItem: () => set(state => {
    if (state.focusItem.type === 'trajectory') {
      // set the line width of the previously selected line to 0
      state.lines[state.focusItem.uuid].width = 0;
      const poses = [];
      if (state.data.trajectories[state.focusItem.uuid].start_location_uuid) {
        poses.push(state.data.trajectories[state.focusItem.uuid].start_location_uuid);
      }
      state.data.trajectories[state.focusItem.uuid].waypoint_uuids.forEach((waypoint_uuid,i)=> {
        poses.push(waypoint_uuid);
      })
      if (state.data.trajectories[state.focusItem.uuid].end_location_uuid) {
        poses.push(state.data.trajectories[state.focusItem.uuid].end_location_uuid); 
      }
      poses.forEach(pose_uuid=>{
        state.items[pose_uuid+'-tag'].color.a = 0;
        state.items[pose_uuid+'-pointer'].color.a = 0;
      })
    } else if (state.focusItem.type === 'waypoint' || state.focusItem.type === 'location') {
      // change the opacity of the previously selected waypoint/pose to 0
      state.items[state.focusItem.uuid+'-tag'].color.a = 0;
      state.items[state.focusItem.uuid+'-pointer'].color.a = 0;
      // Just in case it was being transformed, remove that.
      state.items[state.focusItem.uuid+'-tag'].transformMode = null;
      state.items[state.focusItem.uuid+'-pointer'].transformMode = null;
    }
    state.focusItem = { type: null, uuid: null };
    state.secondaryFocusItem = { type: null, uuid: null };
    // Clear Highlights
    Object.keys(state.items).forEach(itemKey => state.items[itemKey].highlighted = false);
    Object.keys(state.hulls).forEach(hullKey => state.hulls[hullKey].highlighted = false);
  }),
  // the search terms they have entered
  searchTerm: '',
  setSearchTerm: (term) => set(_ => ({ searchTerm: term })),
  clearSearchTerm: () => set(_ => ({ searchTerm: '' })),
  // whether the sim window is expanded to the whole width
  simMode: 'default',
  setSimMode: (mode) => set(_ => ({ simMode: mode })),
  secondaryFocusItem: { type: null, uuid: null },
  setSecondaryFocusItem: (type, uuid) => set(_ => ({ secondaryFocusItem: { type: type, uuid: uuid } })),
  clearSecondaryFocusItem: () => set(_ => ({ secondaryFocusItem: { type: null, uuid: null } })),
  childrenDrawer: false,
  setChildrenDrawer: (input) => set(_ => ({ childrenDrawer: input })),
  clearChildrenDrawer: () => set(_ => ({ childrenDrawer: false })),
  updateFromTfs: (msg) => set(state => {
    msg.transforms.forEach(transformstamped => {
      if (ACTIVE_TFS.indexOf(transformstamped.child_frame_id) >= 0) {
        state.setTF(transformstamped.child_frame_id, state.tfs[transformstamped.child_frame_id])
      }
    })
  }),
  collisionsVisible: false,
  setCollisionsVisible: (visible) => set(state => {
    Object.keys(state.items).filter(key => key.includes('-collision')).forEach(key => state.items[key].color.a = visible ? 1 : 0);
    state.collisionsVisible = visible;
  }),
  setup: (initial_data) => set(state => {
    let setFocusItem = get().setFocusItem;
    Object.keys(initial_data.staticScene)
      .forEach((itemKey) => {
        let item = initial_data.staticScene[itemKey];
        // console.log(item)
        state.items[itemKey] = {
          shape: item.shape,
          name: item.name,
          frame: item.frame,
          position: item.position,
          rotation: item.rotation,
          color: item.color,
          scale: item.scale,
          transformMode: 'inactive',
          highlighted: item.highlighted,
          onClick: (e) => {
            e.stopPropagation();
            setFocusItem('scene', itemKey);
            e.stopPropagation();
          },
          onMove: (transform) => { console.log(transform) }
        };
        if (COLLISION_MESHES[item.shape]) {
          state.items[itemKey + '-collision'] = {
            shape: COLLISION_MESHES[item.shape],
            name: item.name + ' Collision',
            frame: item.frame,
            position: item.position,
            rotation: item.rotation,
            scale: item.scale,
            color: { r: 250, g: 0, b: 0, a: 0 },
            transformMode: 'inactive',
            highlighted: false,
            wireframe: true,
            onClick: (e) => { },
            onMove: (transform) => { console.log(transform) }
          };
        }

      })
    state.tfs = initial_data.tfs;
  })
});
