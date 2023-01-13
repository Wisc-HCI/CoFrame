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
// import PandaDemo from "./Panda_Demo.json";
import { STATUS } from "./Constants";
import useCompiledStore from "./CompiledStore";
import { Timer } from "./Timer";
import { mapValues } from "lodash";
// import { TauriStorage } from "./TauriStorage";

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
  playing: false,
  pause: () => {
    set({playing:false});
    get().clock.setTimescale(0);
  },
  play: (speed) => {
    set({playing:true});
    get().clock.setTimescale(speed ? speed : 1);
  },
  reset: (time) => {
    get().clock._elapsed = time ? time * 1000 : 0;
  },
  tabs: [
    {
      title:'Main',
      id: 'default',
      visible: true,
      blocks: []
    }
  ],
  updateItemDocActive: (id, value) => {
    set((state) => {
      console.log("setting doc active to ", value);
      state.programData = mapValues(state.programData,d=>({
        ...d,
        docActive: id === d.id && value ? true : false
      }))
      // if (value) {
      //   Object.keys(state.programData).forEach((v) => {
      //     if (v === id) {
      //       state.programData[v].docActive = true;
      //     } else if (v !== id && state.programData[v].docActive) {
      //       state.programData[v].docActive = false;
      //     }
      //   });
      //   // state.programData = mapValues(state.programData,(v)=>v.id === id ? {...v,docActive:true} : {...v,docActive:false})
      // } else {
      //   state.programData[id].docActive = false;
      // }

      // state.programData[id].docActive = value;
    });
  },
  activeTab:'default',
});

const immerStore = immer(store);
const subscribeStore = subscribeWithSelector(immerStore);

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
