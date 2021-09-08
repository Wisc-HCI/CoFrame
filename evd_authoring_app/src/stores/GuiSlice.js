import frameStyles from '../frameStyles';
import { INITIAL_SIM, COLLISION_MESHES } from './initialSim';
import { trajectoryToAnimation, poseToAnimation, ANIMATION, ANIMATIONFN } from './animation';
import { typeToKey } from './helpers';

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
    if (state.focusItem.type === 'trajectory') {
      let trajectory = state.data.trajectories[state.focusItem.uuid];
      let locations = state.data.locations;
      let waypoints = state.data.waypoints;
      const {onStart, sequence, onEnd} = trajectoryToAnimation(
        trajectory,
        locations,
        waypoints,
        state.frame,
        get
      );
      state.animation.onStart = onStart;
      state.animation.sequence = sequence;
      state.animation.onEnd = onEnd;
      state.playAnimation();
    }
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
    state.focusItem = { type: type, uuid: uuid };
    // Clear Highlights
    Object.keys(state.items).forEach(itemKey => state.items[itemKey].highlighted = false);
    Object.keys(state.hulls).forEach(hullKey => state.hulls[hullKey].highlighted = false);
    // Clear any animations that are running
    if (state.animation) {
      state.stopAnimation();
    };
    if (type === 'scene' && ROBOT_PARTS.indexOf(uuid) >= 0) {
      ROBOT_PARTS.forEach(part => state.items[part].highlighted = true);
    } else if (type === 'scene' && GRIPPER_PARTS.indexOf(uuid) >= 0) {
      GRIPPER_PARTS.forEach(part => state.items[part].highlighted = true);
    } else if (type === 'scene') {
      state.items[uuid].highlighted = true;
      // useSceneStore.getState().setItemHighlighted(uuid,true)
    } else if (type === 'trajectory') {
      // let setSecondaryFocusItem = get().setSecondaryFocusItem;
      let trajectory = state.data.trajectories[uuid];
      let locations = state.data.locations;
      let waypoints = state.data.waypoints;
      const {onStart, sequence, onEnd} = trajectoryToAnimation(
        trajectory,
        locations,
        waypoints,
        state.frame,
        get
      );
      state.animation.onStart = onStart;
      state.animation.sequence = sequence;
      state.animation.onEnd = onEnd;
      get().playAnimation();
    } else if (type === 'location' || type === 'waypoint') {
      let pose = state.data[typeToKey(type)][uuid];
      const {onStart, sequence, onEnd} = poseToAnimation(pose,state.frame,get)
      state.animation.onStart = onStart;
      state.animation.sequence = sequence;
      state.animation.onEnd = onEnd;
      get().playAnimation();
    }
  }),
  clearFocusItem: () => set(state => {
    state.focusItem = { type: null, uuid: null };
    state.secondaryFocusItem = { type: null, uuid: null };
    // Clear Highlights
    Object.keys(state.items).forEach(itemKey => state.items[itemKey].highlighted = false);
    Object.keys(state.hulls).forEach(hullKey => state.hulls[hullKey].highlighted = false);
    if (state.animation) {
      state.stopAnimation();
    };
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
  animation: {
    state: ANIMATION.INACTIVE,
    next:null,
    onStart:[],
    sequence:{delay:0,changes:[],next:'restart'},
    onEnd:[]
  },
  _advance: () => set(state=>{
    if (state.animation.state === ANIMATION.PAUSED || state.animation.state === ANIMATION.INACTIVE || !state.animation.next) {
      // Do nothing
    } else if (state.animation.next === 'restart') {
      state.animation.next = state.animation.sequence;
      setTimeout(state._advance,state.animation.sequence.delay)
    } else if (state.animation.next === 'end') {
      state.animation.next = state.animation.onEnd;
      setTimeout(state._advance,state.animation.sequence.delay)
    } else {
      state.animation.next.forEach(change=>{
        console.log(change)
        ANIMATIONFN[change.action](state)(...change.data)
      })
      let delay = state.animation.next.next.delay;
      state.animation.next = state.animation.next.next;
      setTimeout(state._advance,delay)
    }
  }),
  pauseAnimation: () => set(state=>{
    state.animation.state = ANIMATION.PAUSED
  }),
  // playAnimation: () => {
  //   let state = get();
  //   if (state.animation.state === ANIMATION.INACTIVE) {
  //     state.animation.onStart.forEach(change=>{
  //       console.log(change)
  //       ANIMATIONFN[change.action](state)(...change.data)
  //     })
  //   } 
  //   if (state.animation.state !== ANIMATION.PLAYING) {
  //     set(state=>{state.animation.state = ANIMATION.PLAYING});
  //     setTimeout(state._advance,100)
  //   }
  // },
  playAnimation: () => set(state=>{
    console.log('playing animation?')
    if (state.animation.state === ANIMATION.INACTIVE) {
      console.log('animation is paused. Running onStart');
      console.log(get().animation.onStart);
      state.animation.onStart.forEach(change=>{
        console.log(change)
        ANIMATIONFN[change.action](state)(...change.data)
      })
    } 
    if (state.animation.state !== ANIMATION.PLAYING) {
      state.animation.state = ANIMATION.PLAYING;
      setTimeout(state._advance,100)
    }
  }),
  // stopAnimation: () => {
  //   let state = get();
  //   state.animation.onEnd.forEach(change=>{
  //     console.log(change)
  //     ANIMATIONFN[change.action](state)(...change.data)
  //   })
  //   set(state=>{
  //     state.animation.state = ANIMATION.INACTIVE;
  //     state.animation.next = null;
  //   })
  // },
  stopAnimation: () => set(state=>{
    state.animation.state = ANIMATION.INACTIVE;
    state.animation.next = null;
    state.animation.onEnd.forEach(change=>{
      console.log(change)
      ANIMATIONFN[change.action](state)(...change.data)
    })
  }),
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
