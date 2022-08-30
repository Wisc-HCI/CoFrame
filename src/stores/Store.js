import create from "zustand";
import shallow from "zustand/shallow";
import { persist, subscribeWithSelector } from "zustand/middleware";
import { immer } from "zustand/middleware/immer";
import { computed } from "zustand-middleware-computed-state";
import { GuiSlice } from "./GuiSlice";
import { ReviewSlice } from "./ReviewSlice";
import { EvdSlice } from "./EvdSlice";
import { RosSlice } from "./RosSlice";
import { ProgrammingSlice } from "simple-vp";
import { ProgrammingSliceOverride } from "./ProgrammingSlice";
import {
  computedSliceCompiledSubscribe,
  computedSliceSubscribe,
} from "./ComputedSlice";
import { SceneSlice } from "robot-scene";
import lodash from "lodash";
import KnifeAssembly from "./Knife_Assembly_Simple_VP.json";
import PandaDemo from "./Panda_Demo.json";
import { STATUS } from "./Constants";
import { performCompileProcess } from "./planner-worker";
import useCompiledStore from "./CompiledStore";
import { Timer } from "./Timer";
import { TauriStorage } from "./TauriStorage";

const store = (set, get) => ({
  loaded: false,
  setLoaded: (loaded) => {
    console.log('new loaded',loaded);
    set({loaded});
    console.log('new loaded test',get().loaded);
  },
  ...SceneSlice(set, get),
  ...ProgrammingSlice(set, get), // default programming slice for simple-vp
  ...ProgrammingSliceOverride(set, get), // overrides data-editing functionality to update pending properties
  ...GuiSlice(set, get),
  ...ReviewSlice(set, get),
  ...EvdSlice(set, get),
  ...RosSlice(set, get),
  clock: new Timer(),
});

const immerStore = immer(store);
// const computedStore = computed(immerStore,computedSlice);
const persistStore = persist(immerStore, {
  name: "coframe-store",
  // partialize: (state) => ({ programData: state.programData, }),
  getStorage: ()=>TauriStorage,
  deserialize: (str) =>
    JSON.parse(str, (key, value) => {
      if (key === "clock") return new Timer();
      return value;
    }),
  partialize: (state) => ({
    loaded: state.loaded,
    programData: state.programData,
    // programSpec: state.programSpec,
    tfs: state.tfs,
    items: state.items,
    lines: state.lines,
    texts: state.texts,
    hulls: state.hulls,
  }),
  onRehydrateStorage: (state) => {
    console.log("hydration starts");
    // optional
    return (state, error) => {
      if (error) {
        console.log("An error happened during hydration. Reloading with Knife Assembly Task", error);
        state.setData(KnifeAssembly);
        state.setLoaded(true);
      } else {
        console.log("hydration finished");
        state.setLoaded(true)
      }
    };
  },
});
const subscribeStore = subscribeWithSelector(persistStore);

const useStore = create(subscribeStore);

useStore.subscribe(
  (state) =>
    lodash.mapValues(state.programData, (value) => {
      return value?.properties?.status
        ? value.properties.status
        : STATUS.PENDING;
    }),
  (currentStatuses, previousStatuses) => {
    if (
      Object.keys(currentStatuses).some(
        (id) =>
          currentStatuses[id] === STATUS.PENDING &&
          previousStatuses[id] !== STATUS.PENDING
      )
    ) {
      console.log("REPLANNING");
      useStore.getState().performCompileProcess();
    }

    // const data = useStore.getState();
    // console.log(data);
    // performCompileProcess({programData:data.programData,objectTypes:data.programSpec.objectTypes})
  },
  { equalityFn: shallow }
);

// Create subscribers for scene data
computedSliceSubscribe(useStore);

if (Object.keys(useStore.getState().programData).length === 0) {
  console.log("Setting with Knife Assembly Task");
  useStore.getState().setData(KnifeAssembly);
  // useStore.persist.rehydrate()
}

computedSliceCompiledSubscribe(useCompiledStore, useStore);

export default useStore;
