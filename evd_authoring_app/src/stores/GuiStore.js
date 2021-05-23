import create from "zustand";
import produce from "immer";

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set) => ({
    frame: 'safety',
    setFrame: (frame) => set((_)=>({frame:frame})),
    editorPane: 'program',
    setEditorPane: (pane) => set((_)=>({editorPane:pane})),
    activeModal: null,
    setActiveModal: (modal) => set((_)=>({activeModal:modal})),
    closeModal: () => set((_)=>({activeModal:null}))
});

const useGuiStore = create(immer(store));

export default useGuiStore;