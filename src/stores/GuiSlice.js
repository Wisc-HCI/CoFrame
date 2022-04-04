import frameStyles from '../frameStyles';
import { DATA_TYPES } from 'simple-vp';
import { remove } from 'lodash';
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

const onClickIgnoredTypes = [
  "meshType",
  "zoneType",
  "inputOutputType"
]

const addFocus = (state, id, add) => {
  // By default clear out the current set
  state.focus.forEach(f => {
    if (state.programData[f]) {
      state.programData[f].selected = false
    }
  });
  // Handle updating the focus given the id and add
  if (state.focus.includes(id)) {
    // the focus already includes this id, so stub the focus there.
    state.focus.length = state.focus.indexOf(id) + 1
  } else if (add) {
    // If add, push the id to the end.
    state.focus.push(id)
  } else {
    // If not adding, replace the focus with a new stack.
    state.focus = [id]
  }
  // Update the nodes to be selected if they appear in focus
  state.focus.forEach(f => {
    if (state.programData[f]) {
      state.programData[f].selected = true
    }
  });
  // Set the active focus to the last item that has
  // an entry in the programData or issues
  state.focus.slice().reverse().some(v=>{
    if (state.programData[v] || state.issues[v]) {
      state.activeFocus = v;
      return true
    } else {
      return false
    }
  })
  return state;
}

const swapLocations = (state, id) => {
  let  hasTrajectory = false;
  let endingFocus = null;
  
  // Search for a trajectory and the location/waypoint that corresponds after it
  state.focus.forEach(entry => {
    if (!hasTrajectory && state.programData[entry]?.type === 'trajectoryType') {
      hasTrajectory = true;
    } else if (hasTrajectory && (state.programData[entry]?.type === 'locationType' || state.programData[entry]?.type === 'waypointType')) {
      endingFocus = entry;
    }
  });

  // If a location/waypoint has been found, deselect it and remove it from focus
  if (endingFocus) {
    state.programData[endingFocus].selected = false;
    remove(state.focus, (entry) => entry === endingFocus);
  }

  // Add new location/waypoint to the focus
  // Note: if a trajectory wasn't found, hasTrajectory will be false and will clear out the old focus
  state = addFocus(state, id, hasTrajectory);

  return state
}

export const GuiSlice = (set, get) => ({
  // EDITOR/SETUP/MAIN
  // The frame specifies the expert (color) frame
  // editorTransform: { x: 0, y: 0, zoom: 1 },
  // setEditorTransform: (transform) => set(_ => ({ editorTransform: transform })),
  // flowInstance: null,
  // setFlowInstance: (instance) => set({ flowInstance: instance }),
  frame: 'safety',
  primaryColor: frameStyles.colors['safety'],
  setFrame: (frame) => set(state => {
    state.frame = frame;
    state.primaryColor = frameStyles.colors[frame];
    let focusIssue = null;
    state.focus.some((focusItem,i)=>{
      if (state.issues[focusItem]) {
        focusIssue = i;
        return true;
      } else {
        return false
      }
    })

    // By default clear out the current set
    state.focus.forEach(f => {
      if (state.programData[f]) {
        state.programData[f].selected = false
      }
    });
    if (focusIssue !== null) {
      state.focus.length = focusIssue - 1
    }
    // Reset new selections based on focus
    state.focus.forEach(f => {
      if (state.programData[f]) {
        state.programData[f].selected = true
      }
    });
    state.activeFocus = state.focus[state.focus.length-1]
  }),
  activeModal: null,
  setActiveModal: (modal) => set(_ => ({ activeModal: modal })),
  closeModal: () => set(_ => ({ activeModal: null })),
  // the activeDrawer specifies the tab in the SetupEditor that is shown
  // activeDrawer: null,
  // setActiveDrawer: (drawer) => set(_ => ({ activeDrawer: drawer })),
  // the focusItem specifies the type and uuid of data to focus on
  focus: [],
  activeFocus: null,
  setActiveFocus: (id) => set(state=>{state.activeFocus=id}),
  addFocusItem: (id, add) => set(state => {
    state = addFocus(state, id, add);
  }),
  clearFocus: () => set(state => {
    console.log('clearing focus')
    state.focus.forEach(f => {
      if (state.programData[f]) {
        state.programData[f].selected = false
      }
    });
    state.focus = []
    state.activeFocus = null
  }),
  // setFocusItem: (type, uuid, transformMode) => set(state=>{
  //   console.log('setFocusItem')
  //   // Clear out previous focusItem highlighting
  //   if (state.focusItem.uuid && state.programData[state.focusItem.uuid]) {
  //     state.programData[state.focusItem.uuid].selected = false
  //   };
  //   // Clear out previous secondaryFocusItem highlighting
  //   if (state.secondaryFocusItem.uuid && state.programData[state.secondaryFocusItem.uuid]) {
  //     state.programData[state.secondaryFocusItem.uuid].selected = false
  //   }
  //   if (uuid && state.programData[uuid]) {
  //     state.programData[uuid].selected = true
  //   };
  //   state.focusItem = { type, uuid, transformMode }
  // }),
  // clearFocusItem: () => set(state => {
  //   console.log('clearFocusItem')
  //   if (state.focusItem.uuid && state.programData[state.focusItem.uuid]) {
  //     state.programData[state.focusItem.uuid].selected = false
  //   }
  //   if (state.secondaryFocusItem.uuid && state.programData[state.secondaryFocusItem.uuid]) {
  //     state.programData[state.secondaryFocusItem.uuid].selected = false
  //   }
  //   state.focusItem = { type: null, uuid: null, transformMode: null };
  //   state.secondaryFocusItem = { type: null, uuid: null, transformMode: null };
  // }),
  // the search terms they have entered
  searchTerm: '',
  setSearchTerm: (term) => set(_ => ({ searchTerm: term })),
  clearSearchTerm: () => set(_ => ({ searchTerm: '' })),
  // whether the sim window is expanded to the whole width
  simMode: 'default',
  setSimMode: (mode) => set(_ => ({ simMode: mode })),
  // secondaryFocusItem: { type: null, uuid: null },
  // setSecondaryFocusItem: (type, uuid, transformMode) => set(state=>{
  //   console.log('setSecondaryFocusItem')
  //   // Clear out previous secondaryFocusItem highlighting
  //   if (state.secondaryFocusItem.uuid && state.programData[state.secondaryFocusItem.uuid]) {
  //     state.programData[state.secondaryFocusItem.uuid].selected = false
  //   }
  //   if (uuid && state.programData[uuid]) {
  //     state.programData[uuid].selected = true
  //   };
  //   state.secondaryFocusItem = { type, uuid, transformMode }
  // }),
  // clearSecondaryFocusItem: () => set(state=>{
  //   console.log('clearSecondaryFocusItem')
  //   if (state.secondaryFocusItem.uuid && state.programData[state.secondaryFocusItem.uuid]) {
  //     state.programData[state.secondaryFocusItem.uuid].selected = false
  //   }
  //   state.secondaryFocusItem = { type: null, uuid: null, transformMode: null };
  // }),
  updateItemSelected: (id, value) => {
    set((state) => {
      console.log({id,value})
      const item = state.programData[id];
      const usedId = (item.dataType === DATA_TYPES.REFERENCE || item.dataType === DATA_TYPES.CALL) ? item.ref : id;
      
      // Clear out current selected
      state.focus.forEach(f => {
        if (state.programData[f]) {
          state.programData[f].selected = false
        }
      });

      // Handle logic of setting focus (replace or trim)
      if (value) {
        if (state.focus.includes(usedId)) {
          state.focus.length = state.focus.indexOf(usedId) + 1
        } else {
          state.focus = [usedId]
        }
      } else {
        if (state.focus.includes(usedId)) {
          state.focus.length = state.focus.indexOf(usedId)
        } else {
          state.focus = []
        }
      }
      

      // Update the selected property for items in focus
      state.focus.forEach(f => {
        if (state.programData[f]) {
          state.programData[f].selected = true
        }
      });

      state.activeFocus = state.focus[state.focus.length-1]
    })
  },
  collisionsVisible: false,
  setCollisionsVisible: (visible) => set(state => {
    state.collisionsVisible = visible;
  }),
  occupancyVisible: false,
  setOccupancyVisible: (visible) => set(state => {
    state.occupancyVisible = visible;
  }),
  tfVisible: false,
  setTfVisible: (visible) => set(state => {
    state.tfVisible = visible;
  }),
  onClick: (id, hidden, event) => set(state => {
    // ignore hidden objects
    // ignore movement for the clicks (translate/rotate)
    // ignore collision meshes (-collision)
    // ignore additional types (onClickIgnoredTypes)
    if (!hidden && 
        !state.focus.includes('translate') && 
        !state.focus.includes('rotate') && 
        !id.includes('-collision') && 
        !onClickIgnoredTypes.includes(state.programData[id]?.type)) {
      if (state.programData[id]?.type === 'linkType') {
        state = addFocus(state, state.programData[id].properties.agent, false);
      } else if (id.endsWith('-pointer')) {
        state = swapLocations(state, id.replace('-pointer', ''));
      } else if (id.endsWith('-tag')) {
        state = swapLocations(state, id.replace('-tag', ''));
      } else {
        state = addFocus(state, id, false);
      }
      event.stopPropagation();
    }
  }),
  onMove: (id, worldTransform, localTransform) => set(state => {
    const focused = state.focus.includes(id);
    const transform = state.focus.includes('translate') 
      ? 'translate' 
      : state.focus.includes('rotate')
      ? 'rotate'
      : 'inactive'
    if (id.includes('pointer') && focused && transform !== 'inactive') {
      state.setPoseTransform(id.replace('-pointer', ''), transform);
    } else if (!id.includes('pointer') && !id.includes('-tag') && focused && transform !== 'inactive') {
     // This isn't correct, we'll want to offset by the object's tf (since we are technically moving the mesh)
     // Similarly, we'll want to compute the quaternion transformation
      state.programData[id].properties.position = localTransform.position;
      state.programData[id].properties.rotation = localTransform.quaternion;
    }
  })
});
