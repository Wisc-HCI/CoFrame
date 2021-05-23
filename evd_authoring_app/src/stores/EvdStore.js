import create from "zustand";
import produce from "immer";

const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set) => ({
    program: null,
    setProgram: (program) => set((_)=>({program:program}))
});

const useEvdStore = create(immer(store));

export default useEvdStore;