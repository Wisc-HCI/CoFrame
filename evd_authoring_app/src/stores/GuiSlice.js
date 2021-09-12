import frameStyles from '../frameStyles';

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
  }),
  clearFocusItem: () => set(state => {
    state.focusItem = { type: null, uuid: null };
    state.secondaryFocusItem = { type: null, uuid: null };
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
    state.collisionsVisible = visible;
  })
});
