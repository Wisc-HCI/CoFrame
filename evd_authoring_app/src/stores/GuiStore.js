import create from "zustand";
import produce from "immer";
import frameStyles from '../frameStyles';
import { typeToKey } from "./EvdStore";

const EDITOR_TYPES = ['primitive','skill','program','trajectory'];

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set) => ({
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
    setEditorPane: (pane) => set((_)=>({editorPane:pane})),
    // the activeModal specifies the type of modal shown
    activeModal: null,
    setActiveModal: (modal) => set((_)=>({activeModal:modal})),
    closeModal: () => set((_)=>({activeModal:null})),
    // the setupTab specifies the tab in the SetupEditor that is shown
    setupTab: 'locations',
    setSetupTab: (tab) => set((_)=>({setupTab:tab})),
    // the focusItem specifies the type and uuid of data to focus on
    focusItem: {type:null,uuid:null},
    setFocusItem: (type,uuid) => set((_)=>({
      focusItem:{type:type,uuid:uuid},
      editorPane:EDITOR_TYPES.indexOf(type)>=0?'editor':'setup',
      setupTab:EDITOR_TYPES.indexOf(type)>=0?'locations':typeToKey(type)
    })),
    clearFocusItem: () => set((_)=>({focusItem:{type:null,uuid:null},secondaryFocusItem:{type:null,uuid:null}})),
    // the search terms they have entered
    searchTerm: '',
    setSearchTerm: (term) => set((_)=>({searchTerm:term})),
    clearSearchTerm: () => set((_)=>({searchTerm:''})),
    // whether the sim window is expanded to the whole width
    simMode: 'default',
    setSimMode: (mode) => set((_)=>({simMode:mode})),
    // for program editor, dragged item uuid and metadata
    dragItem: null,
    setDragItem: (item) => set((_)=>({dragItem:item})),
    clearDragItem: () => set((_)=>({dragItem:null})),
    dragHoverItem: null,
    setDragHoverItem: (item) => set((_)=>({dragHoverItem:item})),
    clearDragHoverItem: () => set((_)=>({dragHoverItem:null})),
    secondaryFocusItem: {type:null,uuid:null},
    setSecondaryFocusItem: (type,uuid) => set((_)=>({secondaryFocusItem:{type:type,uuid:uuid}})),
    clearSecondaryFocusItem: () => set((_)=>({secondaryFocusItem:{type:null,uuid:null}})),
    childrenDrawer: false,
    setChildrenDrawer: (input) => set((_)=>({childrenDrawer: input})),
    clearChildrenDrawer : () => set((_)=>({childrenDrawer: false}))
});

const useGuiStore = create(immer(store));

export default useGuiStore;
