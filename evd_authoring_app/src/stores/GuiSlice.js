import frameStyles from '../frameStyles';
import { sceneSetItem, sceneSetTF } from "./helpers";
import { INITIAL_SIM, COLLISION_MESHES } from './initialSim';
import { clearHighlights, // clearTempObjects, createTrajectory, 
    highlightGripper, highlightRobot, highlightSceneItem, sceneSetTfs } from './helpers';
import { trajectoryToAnimation } from './animation';
// import { useSceneStore } from 'robot-scene';

// const EDITOR_TYPES = ['primitive','skill','program','trajectory'];

const ROBOT_PARTS = Object.keys(INITIAL_SIM.staticScene).filter(v=>v.includes('robot'));
const GRIPPER_PARTS = Object.keys(INITIAL_SIM.staticScene).filter(v=>v.includes('gripper'));

const ACTIVE_TFS = [
    'simulated_base_link',
    'simulated_shoulder_link',
    'simulated_upper_arm_link',
    'simulated_forearm_link',
    'simulated_wrist_1_link',
    'simulated_wrist_2_link',
    'simulated_wrist_3_link'
  ]

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
    // the activeModal specifies the type of modal shown
    activeModal: null,
    setActiveModal: (modal) => set(_=>({activeModal:modal})),
    closeModal: () => set(_=>({activeModal:null})),
    // the activeDrawer specifies the tab in the SetupEditor that is shown
    activeDrawer: null,
    setActiveDrawer: (drawer) => set(_=>({activeDrawer:drawer})),
    // the focusItem specifies the type and uuid of data to focus on
    focusItem: {type:null,uuid:null},
    setFocusItem: (type,uuid) => set(state=>{
      state.focusItem = {type:type,uuid:uuid};
      // if (type !== 'scene') {
      //   // state.actuve = EDITOR_TYPES.indexOf(type)>=0?'editor':'setup'
      //   state.activeDrawer = EDITOR_TYPES.indexOf(type)>=0?null:typeToKey(type)
      // }
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
    secondaryFocusItem: {type:null,uuid:null},
    setSecondaryFocusItem: (type,uuid) => set(_=>({secondaryFocusItem:{type:type,uuid:uuid}})),
    clearSecondaryFocusItem: () => set(_=>({secondaryFocusItem:{type:null,uuid:null}})),
    childrenDrawer: false,
    setChildrenDrawer: (input) => set(_=>({childrenDrawer: input})),
    clearChildrenDrawer : () => set(_=>({childrenDrawer: false})),
    animation: null,
    playAnimation: () => {get().animation?.play()},
    pauseAnimation: () => {get().animation?.pause()},
    endAnimation: () => {get().animation?.end()},
    updateFromTfs: (msg) => set(state=>{
        msg.transforms.forEach(transformstamped=>{
            if (ACTIVE_TFS.indexOf(transformstamped.child_frame_id) >= 0) {
                sceneSetTF(transformstamped.child_frame_id,state.tfs[transformstamped.child_frame_id])
            }
        })
    }),
    setup: (initial_data) => {
        Object.keys(initial_data.staticScene)
            .forEach((itemKey)=>{
                let item = initial_data.staticScene[itemKey];
                sceneSetItem(itemKey,{
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
                        get().setFocusItem('scene',itemKey);
                        e.stopPropagation();},
                    onMove: (transform) => {console.log(transform)}
                });
                if (COLLISION_MESHES[item.shape]) {
                  sceneSetItem(itemKey+'-collision',{
                    shape: COLLISION_MESHES[item.shape],
                    name: item.name+' Collision',
                    frame: item.frame,
                    position: item.position,
                    rotation: item.rotation,
                    scale: item.scale,
                    color: {r: 250, g: 0, b: 0, a: 0.3},
                    transformMode: 'inactive',
                    highlighted: false,
                    wireframe: true,
                    onClick: (e)=>{},
                    onMove: (transform) => {console.log(transform)}
            });
                }
                
            })
            sceneSetTfs(initial_data.tfs);
        }
});
