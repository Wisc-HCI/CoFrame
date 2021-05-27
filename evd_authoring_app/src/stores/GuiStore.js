import create from "zustand";
import produce from "immer";

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set) => ({
    // The frame specifies the expert (color) frame
    frame: 'safety',
    setFrame: (frame) => set((_)=>({frame:frame})),
    // The editorPane specifies the section of the ProgramTile that is shown
    editorPane: 'program',
    setEditorPane: (pane) => set((_)=>({editorPane:pane})),
    // the activeModal specifies the type of modal shown
    activeModal: null,
    setActiveModal: (modal) => set((_)=>({activeModal:modal})),
    closeModal: () => set((_)=>({activeModal:null})),
    // the focusItem specifies the type and uuid of data to focus on
    focusItem: {type:null,uuid:null},
    setFocusItem: (type,uuid) => set((_)=>({focusItem:{type:type,uuid:uuid}})),
    clearFocusItem: () => set((_)=>({focusItem:{type:null,uuid:null}}))
});

const useGuiStore = create(immer(store));

export default useGuiStore;