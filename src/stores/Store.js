import { create } from "zustand";
import { shallow } from "zustand/shallow";
import { subscribeWithSelector } from "zustand/middleware";
import { immer } from "zustand/middleware/immer";
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
// import KnifeAssembly from "./Knife_Assembly_Simple_VP_UR5.json";
import KnifeAssembly from "./Knife_Assembly_Example.json";
// import KnifeAssemblyCompiled from "./UR5_Compiled_Knife_Assembly.json";
// import KnifeAssembly from './Prime_Process_2_v29.json';
import KnifeAssemblyCompiled from "./Knife_Assembly_Example_Compiled.json";
import TestProgram2 from "./TestProgram2.json";
// import PandaDemo from "./Panda_Demo.json";
import { STATUS } from "./Constants";
import useCompiledStore from "./CompiledStore";
import { Timer } from "./Timer";
import { mapValues } from "lodash";
import { ProgramStoreSlice } from "./ProgramStoreSlice";
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
  ...ProgramStoreSlice(set, get),
  clock: new Timer(),
  playing: true,
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
    });
  },
  activeTab:'default',
  reviewExpanded: true,
  setReviewExpanded: (expanded) => set(state=>{state.reviewExpanded = expanded})
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
  // Load all the programs
  useStore.getState().addProgramData("KnifeAssembly", KnifeAssembly, {});
  // useStore.getState().addProgramData("testProgram2", TestProgram2, {});

  // Set the starting program
  useCompiledStore.setState({});
  useStore.getState().setData(KnifeAssembly);
  // useStore.persist.rehydrate()
}

computedSliceCompiledSubscribe(useCompiledStore, useStore);

export default useStore;
