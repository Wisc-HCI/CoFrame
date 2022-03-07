// import create from 'zustand'
import create from 'zustand';
import shallow from 'zustand/shallow'
import { subscribeWithSelector } from 'zustand/middleware'
import { computed } from 'zustand-middleware-computed-state'
import produce from "immer";
// import { persist } from "zustand/middleware";
import {GuiSlice} from './GuiSlice';
import {ReviewSlice} from './ReviewSlice';
import {EvdSlice} from './EvdSlice';
import {RosSlice} from './RosSlice';
import {ProgrammingSlice} from 'simple-vp';
import { ProgrammingSliceOverride } from './ProgrammingSlice';
// import {SceneSlice} from 'robot-scene';
import {computedSlice} from './ComputedSlice';
// import {WatchedSlice} from './WatchedSlice';
// import { computed } from 'zustand-middleware-computed-state';
// import {SimSlice} from './SimSlice';
import lodash from 'lodash';
// import { INITIAL_SIM } from "./initialSim";
// import fakeEvdData from './fakeEvdData';
// import KNIFE_TASK from './knifeTask';
// import KnifeAssembly from './Knife_Assembly.json';
// import KnifeAssembly from './Knife_Assembly_Refactor.json';
import KnifeAssembly from './Knife_Assembly_Simple_VP.json';
import { STATUS } from './Constants';
// import { Solver } from '@people_and_robots/lively_tk';
// import {ur3e} from './ur3e.xml';
// import {buffer} from "@people_and_robots/lively_tk";
// import {greet} from './hello_wasm'
// import { primitiveTypes } from './templates';

const immer = (config) => (set, get, api) =>
  config(
    (partial, replace) => {
      const nextState =
        typeof partial === "function" ? produce(partial) : partial;
      return set(nextState, replace);
    },
    get,
    api
);

const store = (set, get) => ({
    ...ProgrammingSlice(set,get), // default programming slice for simple-vp
    ...ProgrammingSliceOverride(set,get), // overrides data-editing functionality to update pending properties
    ...GuiSlice(set,get),
    ...ReviewSlice(set,get),
    ...EvdSlice(set,get),
    ...RosSlice(set,get),
})

const useStore = create(subscribeWithSelector(computed(immer(store),computedSlice)));


useStore.getState().setData(KnifeAssembly);

useStore.subscribe(store=>
  lodash.mapValues(store.programData,(value)=>{
    return value?.properties?.status ? value.properties.status : STATUS.PENDING
  }),
  ()=>{
    console.log("REPLANNING")
    useStore.getState().performPlanProcess()
  },
  {equalityFn:shallow}
)

useStore.getState().performPlanProcess()

// useStore.getState().loadSolver();
// useStore.getState().setSolver()
console.log(useStore.getState())
// console.log(KNIFE_TASK.environment.trajectories[0])
useStore.getState().setUrl('ws://localhost:9090');

export default useStore;