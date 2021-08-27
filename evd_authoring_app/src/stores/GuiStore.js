import create from "zustand";
import produce from "immer";
import frameStyles from '../frameStyles';
import { typeToKey } from "./EvdStore";
import {useSceneStore} from 'robot-scene';
import { INITIAL_SIM } from "./initialSim";

const EDITOR_TYPES = ['primitive','skill','program','trajectory'];

const ACTIVE_TFS = [
  'simulated_base_link',
  'simulated_shoulder_link',
  'simulated_upper_arm_link',
  'simulated_forearm_link',
  'simulated_wrist_1_link',
  'simulated_wrist_2_link',
  'simulated_wrist_3_link'
]

const VISUAL_FOCUS_TYPES = ['trajectory','scene']

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set,get) => ({
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
      if (type === 'scene') {
        useSceneStore.getState().setItemHighlighted(uuid,true)
      } else if (type === 'trajectory') {
        console.log('show trajectory')
      }
    }),
    clearFocusItem: () => set(_=>({focusItem:{type:null,uuid:null},secondaryFocusItem:{type:null,uuid:null}})),
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
    // SIMULATION
    updateInterval:null,
    tfs:{},
    staticScene: {},
    animationObjects: {},
    controllers: {},
    updateFromTfs: (msg) => set(state=>{
        msg.transforms.forEach(transformstamped=>{
            if (ACTIVE_TFS.indexOf(transformstamped.child_frame_id) >= 0) {
                state.tfs[transformstamped.child_frame_id].translation = transformstamped.transform.translation;
                state.tfs[transformstamped.child_frame_id].rotation = transformstamped.transform.rotation;
                useSceneStore.getState().setTf(transformstamped.child_frame_id,get().tfs[transformstamped.child_frame_id])
            }
        })
    }),
    setup: ()=>set((state)=>{
        state.staticScene = INITIAL_SIM.staticScene;
        state.tfs = INITIAL_SIM.tfs;
        let items = {};
        Object.keys(INITIAL_SIM.staticScene)
            .forEach((itemKey)=>{
                let item = INITIAL_SIM.staticScene[itemKey];
                items[itemKey] = {
                    shape: item.visual,
                    name: item.name,
                    frame: item.frame,
                    position: item.position,
                    rotation: item.rotation,
                    color: item.color,
                    scale: item.scale,
                    transformMode: 'inactive',
                    highlighted: item.highlighted,
                    onClick: (e) => {e.stopPropagation();console.log(itemKey);get().setFocusItem('scene',itemKey)},
                    onMove: (transform) => {console.log(transform)}
                };
                if (item.showCollision) {
                    items[itemKey+'-collision'] = {
                        shape: item.collision,
                        name: item.name+' Collision',
                        frame: item.frame,
                        position: item.position,
                        rotation: item.rotation,
                        scale: { x: 1, y: 1, z: 1 },
                        color: {r: 250, g: 50, b: 50, a: 0.5},
                        transformMode: 'inactive',
                        highlighted: false,
                        onClick: (e)=>{e.stopPropagation();get().setFocusItem('scene',itemKey+'-collision')},
                        onMove: (transform) => {console.log(transform)}
                    };
                }
            });
            useSceneStore.getState().setItems(items);
            useSceneStore.getState().setTfs(INITIAL_SIM.tfs);
            useSceneStore.getState().clearLines();
            state.updateInterval = setInterval(get().sync,100)
    }),
    sync: ()=>(state=>{
        console.log('update')
        Object.keys(state.staticScene)
            .forEach((itemKey)=>{
                let item = get().staticScene[itemKey];
                if (item.pending) {
                    useSceneStore.getState().setItem(itemKey,{
                        shape: item.visual,
                        name: item.name,
                        frame: item.frame,
                        position: item.position,
                        rotation: item.rotation,
                        color: item.color,
                        scale: item.scale,
                        transformMode: 'inactive',
                        highlighted: item.highlighted,
                        onClick: () => {console.log(itemKey)},
                        onMove: (transform) => {console.log(transform)}
                    })
                }
                if (item.pending && item.showCollision) {
                    useSceneStore.getState().setItem(itemKey+'-collision',{
                        shape: item.collision,
                        name: item.name+' Collision',
                        frame: item.frame,
                        position: item.position,
                        rotation: item.rotation,
                        scale: { x: 1, y: 1, z: 1 },
                        color: {r: 250, g: 50, b: 50, a: 0.5},
                        transformMode: 'inactive',
                        highlighted: false,
                        onClick: () => {console.log(itemKey)},
                        onMove: (transform) => {console.log(transform)}
                    })
                } else if (item.pending && !item.showCollision) {
                    useSceneStore.getState().removeItem(itemKey+'-collision')
                }
        });
    })
});

const useGuiStore = create(immer(store));

export default useGuiStore;
