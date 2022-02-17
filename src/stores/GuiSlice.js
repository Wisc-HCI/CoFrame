import frameStyles from '../frameStyles';
import { DATA_TYPES } from 'simple-vp';
// import { INITIAL_SIM, COLLISION_MESHES } from './initialSim';

// const ROBOT_PARTS = Object.keys(INITIAL_SIM.staticScene).filter(v => v.includes('robot'));
// const GRIPPER_PARTS = Object.keys(INITIAL_SIM.staticScene).filter(v => v.includes('gripper'));

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
  editorTransform: { x: 0, y: 0, zoom: 1 },
  setEditorTransform: (transform) => set(_ => ({ editorTransform: transform })),
  flowInstance: null,
  setFlowInstance: (instance) => set({ flowInstance: instance }),
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
  activeModal: null,
  setActiveModal: (modal) => set(_ => ({ activeModal: modal })),
  closeModal: () => set(_ => ({ activeModal: null })),
  // the activeDrawer specifies the tab in the SetupEditor that is shown
  activeDrawer: null,
  setActiveDrawer: (drawer) => set(_ => ({ activeDrawer: drawer })),
  // the focusItem specifies the type and uuid of data to focus on
  focusItem: { type: null, uuid: null, transformMode: null },
  setFocusItem: (type, uuid, transformMode) => set(state=>{
    console.log('setFocusItem')
    // Clear out previous focusItem highlighting
    if (state.focusItem.uuid && state.programData[state.focusItem.uuid]) {
      state.programData[state.focusItem.uuid].selected = false
    };
    // Clear out previous secondaryFocusItem highlighting
    if (state.secondaryFocusItem.uuid && state.programData[state.secondaryFocusItem.uuid]) {
      state.programData[state.secondaryFocusItem.uuid].selected = false
    }
    if (uuid && state.programData[uuid]) {
      state.programData[uuid].selected = true
    };
    state.focusItem = { type, uuid, transformMode }
  }),
  clearFocusItem: () => set(state => {
    console.log('clearFocusItem')
    if (state.focusItem.uuid && state.programData[state.focusItem.uuid]) {
      state.programData[state.focusItem.uuid].selected = false
    }
    if (state.secondaryFocusItem.uuid && state.programData[state.secondaryFocusItem.uuid]) {
      state.programData[state.secondaryFocusItem.uuid].selected = false
    }
    state.focusItem = { type: null, uuid: null, transformMode: null };
    state.secondaryFocusItem = { type: null, uuid: null, transformMode: null };
  }),
  // the search terms they have entered
  searchTerm: '',
  setSearchTerm: (term) => set(_ => ({ searchTerm: term })),
  clearSearchTerm: () => set(_ => ({ searchTerm: '' })),
  // whether the sim window is expanded to the whole width
  simMode: 'default',
  setSimMode: (mode) => set(_ => ({ simMode: mode })),
  secondaryFocusItem: { type: null, uuid: null },
  setSecondaryFocusItem: (type, uuid, transformMode) => set(state=>{
    console.log('setSecondaryFocusItem')
    // Clear out previous secondaryFocusItem highlighting
    if (state.secondaryFocusItem.uuid && state.programData[state.secondaryFocusItem.uuid]) {
      state.programData[state.secondaryFocusItem.uuid].selected = false
    }
    if (uuid && state.programData[uuid]) {
      state.programData[uuid].selected = true
    };
    state.secondaryFocusItem = { type, uuid, transformMode }
  }),
  clearSecondaryFocusItem: () => set(state=>{
    console.log('clearSecondaryFocusItem')
    if (state.secondaryFocusItem.uuid && state.programData[state.secondaryFocusItem.uuid]) {
      state.programData[state.secondaryFocusItem.uuid].selected = false
    }
    state.secondaryFocusItem = { type: null, uuid: null, transformMode: null };
  }),
  updateItemSelected: (id, value) => {
    set((state) => {
      const item = state.programData[id];
      const usedId = (item.dataType === DATA_TYPES.REFERENCE || item.dataType === DATA_TYPES.CALL) ? item.ref : id;
      state.programData[usedId].selected = value;
      if (value) {
        state.focusItem = { type:'data', uuid:usedId, transformMode:null }
      } else {
        state.focusItem = { type:null, uuid:null, transformMode:null }
      }
      if (state.secondaryFocusItem.uuid) {
        state.secondaryFocusItem = { type:null, uuid:null, transformMode:null }
      }
    })
  },
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
    state.collisionsVisible = visible;
  }),
  occupancyVisible: false,
  setOccupancyVisible: (visible) => set(state => {
    state.occupancyVisible = visible;
  }),
});
