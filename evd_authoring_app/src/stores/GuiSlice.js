import frameStyles from '../frameStyles';
import { typeToKey } from "./helpers";
import { INITIAL_SIM } from './initialSim';
import { clearHighlights, // clearTempObjects, createTrajectory, 
    highlightGripper, highlightRobot, highlightSceneItem } from './helpers';
import { trajectoryToAnimation } from './animation';
// import { useSceneStore } from 'robot-scene';

const EDITOR_TYPES = ['primitive','skill','program','trajectory'];

const ROBOT_PARTS = Object.keys(INITIAL_SIM.staticScene).filter(v=>v.includes('robot'));
const GRIPPER_PARTS = Object.keys(INITIAL_SIM.staticScene).filter(v=>v.includes('gripper'));



export const GuiSlice = (set,get) => ({
    // EDITOR/SETUP/MAIN
    // The frame specifies the expert (color) frame
    frame: 'safety',
    primaryColor: frameStyles.colors['safety'],
    setFrame: (frame) => set(state=>{
      state.frame = frame;
      state.primaryColor = frameStyles.colors[frame];
      if (state.secondaryFocusItem.type === 'issue') {
        state.focusItem = {type:null,uuid:null};
        state.secondaryFocusItem = {type:null,uuid:null};
      }
      if (state.focusItem.type === 'trajectory') {
        let setSecondaryFocusItem = get().setSecondaryFocusItem;
        let trajectory = state.data.trajectories[state.focusItem.uuid];
        let locations = state.data.locations;
        let waypoints = state.data.waypoints;
        state.animation = trajectoryToAnimation(
            trajectory,
            locations,
            waypoints,
            state.frame,
            setSecondaryFocusItem
        );
        console.log(state.animation);
        state.animation.start();

      }
    }),
    // The editorPane specifies the section of the ProgramTile that is shown
    editorPane: 'setup',
    setEditorPane: (pane) => set(_=>({editorPane:pane})),
    // the activeModal specifies the type of modal shown
    activeModal: null,
    setActiveModal: (modal) => set(_=>({activeModal:modal})),
    closeModal: () => set(_=>({activeModal:null})),
    // the setupTab specifies the tab in the SetupEditor that is shown
    setupTab: 'locations',
    setSetupTab: (tab) => set(_=>({setupTab:tab})),
    // the focusItem specifies the type and uuid of data to focus on
    focusItem: {type:null,uuid:null},
    setFocusItem: (type,uuid) => set(state=>{
      state.focusItem = {type:type,uuid:uuid};
      if (type !== 'scene') {
        state.editorPane = EDITOR_TYPES.indexOf(type)>=0?'editor':'setup'
        state.setupTab = EDITOR_TYPES.indexOf(type)>=0?'locations':typeToKey(type)
      }
      clearHighlights();
      // Clear any animations that are running
      if (state.animation) {
          state.animation.end();
      };
      if (type === 'scene' && ROBOT_PARTS.indexOf(uuid) >= 0) {
        highlightRobot();
      } else if (type === 'scene' && GRIPPER_PARTS.indexOf(uuid) >= 0) {
        highlightGripper();
      } else if (type === 'scene') {
        highlightSceneItem(uuid);
        // useSceneStore.getState().setItemHighlighted(uuid,true)
      } else if (type === 'trajectory') {
        let setSecondaryFocusItem = get().setSecondaryFocusItem;
        let trajectory = state.data.trajectories[uuid];
        let locations = state.data.locations;
        let waypoints = state.data.waypoints;
        state.animation = trajectoryToAnimation(
            trajectory,
            locations,
            waypoints,
            state.frame,
            setSecondaryFocusItem
        );
        console.log(state.animation)
        state.animation.start();
      }
    }),
    clearFocusItem: () => set(state=>{
        state.focusItem = {type:null,uuid:null};
        state.secondaryFocusItem = {type:null,uuid:null};
        clearHighlights();
        if (state.animation) {
            state.animation.end();
        };
    }),
    // the search terms they have entered
    searchTerm: '',
    setSearchTerm: (term) => set(_=>({searchTerm:term})),
    clearSearchTerm: () => set(_=>({searchTerm:''})),
    // whether the sim window is expanded to the whole width
    simMode: 'default',
    setSimMode: (mode) => set(_=>({simMode:mode})),
    // for program editor, dragged item uuid and metadata
    dragItem: null,
    setDragItem: (item) => set(_=>({dragItem:item})),
    clearDragItem: () => set(_=>({dragItem:null})),
    dragHoverItem: null,
    setDragHoverItem: (item) => set(_=>({dragHoverItem:item})),
    clearDragHoverItem: () => set(_=>({dragHoverItem:null})),
    secondaryFocusItem: {type:null,uuid:null},
    setSecondaryFocusItem: (type,uuid) => set(_=>({secondaryFocusItem:{type:type,uuid:uuid}})),
    clearSecondaryFocusItem: () => set(_=>({secondaryFocusItem:{type:null,uuid:null}})),
    childrenDrawer: false,
    setChildrenDrawer: (input) => set(_=>({childrenDrawer: input})),
    clearChildrenDrawer : () => set(_=>({childrenDrawer: false})),
    animation: null,
    playAnimation: () => {get().animation?.play()},
    pauseAnimation: () => {get().animation?.pause()},
    endAnimation: () => {get().animation?.end()}
});
