import create from "zustand";
import produce from "immer";

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set) => ({
    frame: 'safety',
    setFrame: (frame) => set({frame:frame})
});

const useGuiStore = create(immer(store));

export default useGuiStore;