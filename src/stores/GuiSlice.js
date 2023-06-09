import frameStyles from '../frameStyles';
import { DATA_TYPES, instanceTemplateFromSpec } from 'simple-vp';
import { remove } from 'lodash';
import { STATUS } from './Constants';
import { mapValues } from 'lodash';
import { round } from 'number-precision';
import { generateUuid } from './generateUuid';

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

const createNewInstance = (state, instanceType) => {
  const id = generateUuid(instanceType);
  const template = {
    ...instanceTemplateFromSpec(
      instanceType,
      state.programSpec.objectTypes[instanceType],
      false
    ),
    id,
    dataType: DATA_TYPES.INSTANCE,
  };
  state.programData[id] = template;
  return id;
}

const swapLocations = (state, id) => {
  let  hasTrajectory = false;
  let endingFocus = null;
  
  // Search for a trajectory and the location/waypoint that corresponds after it
  state.focus.forEach(entry => {
    if (!hasTrajectory && (state.programData[entry]?.type === 'trajectoryType' || state.programData[entry]?.type === 'moveTrajectoryType')) {
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

const updatePartialSceneState = (state, sceneKey, obj) => {
  const keys = obj ? Object.keys(obj) : [];
  keys.forEach(key => {
    const objKeys = Object.keys(obj[key]);
    objKeys.forEach(objKey => {
      state[sceneKey][key][objKey] = obj[key][objKey];
    });
  });

  return state;
}

const updateSceneState = (state, sceneKey, obj) => {
  const keys = obj ? Object.keys(obj) : [];
  keys.forEach(key => {
      state[sceneKey][key] = obj[key];
    });

  return state;
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
    // console.log('clearing focus')
    state.focus.forEach(f => {
      if (state.programData[f]) {
        state.programData[f].selected = false
      }
    });
    state.focus = []
    state.activeFocus = null
  }),
  searchTerm: '',
  setSearchTerm: (term) => set(_ => ({ searchTerm: term })),
  clearSearchTerm: () => set(_ => ({ searchTerm: '' })),
  // whether the sim window is expanded to the whole width
  viewMode: 'default',
  setViewMode: (mode) => set(_ => ({ viewMode: mode })),
  updateItemSelected: (id, value) => {
    set((state) => {
      if (!state.captureFocus) {
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
      }
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
  fixtureClickable: false,
  setFixtureClickable: (clickable) => set(state => {
    state.fixtureClickable = clickable;
  }),
  tfVisible: false,
  setTfVisible: (visible) => set(state => {
    state.tfVisible = visible;
  }),
  robotPreviewVisible: false,
  applyPartialSceneUpdate: ({tfs, items, lines, hulls, texts}) => set(state => {
    state = updatePartialSceneState(state, 'tfs', tfs);
    state = updatePartialSceneState(state, 'items', items);
    state = updatePartialSceneState(state, 'lines', lines);
    state = updatePartialSceneState(state, 'hulls', hulls);
    state = updatePartialSceneState(state, 'texts', texts);
  }),
  applySceneUpdate: ({tfs, items, lines, hulls, texts}) => set(state => {
    state = updateSceneState(state, 'tfs', tfs);
    state = updateSceneState(state, 'items', items);
    state = updateSceneState(state, 'lines', lines);
    state = updateSceneState(state, 'hulls', hulls);
    state = updateSceneState(state, 'texts', texts);
  }),
  setSceneState: ({tfs, items, lines, hulls, texts}) => set(state => {
    state.tfs = tfs;
    state.items = items;
    state.lines = lines;
    state.hulls = hulls;
    state.texts = texts;
  }),
  partialSceneState: ({tfs, items}) => set(state => {
    Object.keys(tfs).forEach(key => {
      state.tfs[key] = { ...state.tfs[key], ...tfs[key] };
    });

    Object.keys(items).forEach(key => {
      state.items[key] = {...state.items[key], ...items[key]};
    });
  }),
  setRobotPreviewVisible: (visible) => set(state => {
    state.robotPreviewVisible = visible;
  }),
  updateCompleteGoals: (update) => set(state => {
    Object.keys(update).forEach(key => {
      state.programData[key].properties.isComplete = update[key];
    });
  }),
  onClick: (id, hidden, transform, event) => set(state => {
    // ignore hidden objects
    // ignore movement for the clicks (translate/rotate)
    // ignore collision meshes (-collision)
    // ignore additional types (onClickIgnoredTypes)
    if (!hidden && 
        !state.focus.includes('translate') && 
        !state.focus.includes('rotate') && 
        !state.captureFocus && 
        !id.includes('-collision') && 
        !onClickIgnoredTypes.includes(state.programData[id]?.type)) {
      if (state.programData[id]?.type === 'linkType') {
        state = addFocus(state, state.programData[id].properties.agent, false);
      } else if (id.endsWith('-pointer')) {
        state = swapLocations(state, id.replace('-pointer', ''));
      } else if (id.endsWith('-tag')) {
        state = swapLocations(state, id.replace('-tag', ''));
      } else if (id.startsWith('ghost-') || id.endsWith("-viz")) {
        // Grasp obj
        // let graspId = id.split('--')[3];
        // let graspObj = state.programData[graspId];

        // Create a location type
        let newId = createNewInstance(state, "locationType");
        let worldPose = transform.world;

        // Update that location with the position of the grasp point
        state.programData[newId].properties.position.x = worldPose.position.x;
        state.programData[newId].properties.position.y = worldPose.position.y;
        state.programData[newId].properties.position.z = worldPose.position.z;
        state.programData[newId].properties.rotation.x = worldPose.rotation.x;
        state.programData[newId].properties.rotation.y = worldPose.rotation.y;
        state.programData[newId].properties.rotation.z = worldPose.rotation.z;
        state.programData[newId].properties.rotation.w = worldPose.rotation.w;

        // update that location with the name of the grasp point
        // state.programData[newId].name = "Loc: " + graspObj.name;
        state.programData[newId].name = "Grasp Point";
      } else if (state.programData[id]?.type === 'fixtureType') {
        if (state.fixtureClickable) {
          state = addFocus(state, id, false);
        }
      } else {
        state = addFocus(state, id, false);
      }
      event.stopPropagation();
    }
  }),
  moveFlag: 'default',
  onMove: (id, source, worldTransform, localTransform) => {
    if (get().moveFlag === 'default') {
      set(state => {
        console.log('ON MOVE',{id,source,worldTransform,localTransform})
        const filteredId = 
        id.includes('-pointer') ? id.replace('-pointer', '') : 
        id.includes('-tag') ? id.replace('-tag', '') : id;
        const focused = state.focus.includes(filteredId);
        const transform = state.focus.includes('translate') 
          ? 'translate' 
          : state.focus.includes('rotate')
          ? 'rotate'
          : 'inactive'
        if (id.includes('-pointer') && focused && transform !== 'inactive') {
          // state.setPoseTransform(filteredId, transform);
          state.programData[filteredId].properties.position = mapValues(localTransform.position,(v)=>round(v,3));
          state.programData[filteredId].properties.rotation = mapValues(localTransform.quaternion,(v)=>round(v,3));
          state.programData[filteredId].properties.status = STATUS.PENDING;
        } else if (!id.includes('pointer') && !id.includes('-tag') && focused && transform !== 'inactive') {
         // This isn't correct, we'll want to offset by the object's tf (since we are technically moving the mesh)
         // Similarly, we'll want to compute the quaternion transformation
          state.programData[filteredId].properties.position = mapValues(localTransform.position,(v)=>round(v,3));;
          state.programData[filteredId].properties.rotation = mapValues(localTransform.quaternion,(v)=>round(v,3));
          state.programData[filteredId].properties.status = STATUS.PENDING;
        }
      })
    } else {
      get().customMoveHook(id, source, worldTransform, localTransform)
    }
  },
  customMoveHook: ()=>{},
  setCustomMoveHook: (newHook) => set(state=>{
    if (newHook) {
      state.customMoveHook = newHook;
      state.moveFlag = 'custom';
    } else {
      state.customMoveHook = ()=>{},
      state.moveFlag = 'default';
    }
  }),
  captureFocus: false,
  setCaptureFocus: (capture) => set(state=>{state.captureFocus = capture})
});
