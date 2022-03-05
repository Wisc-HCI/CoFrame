// import create from 'zustand'
import create from 'zustand';
// import { subscribeWithSelector } from 'zustand/middleware'
import { computed } from 'zustand-middleware-computed-state'
import produce from "immer";
import { persist } from "zustand/middleware";
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

// import { INITIAL_SIM } from "./initialSim";
// import fakeEvdData from './fakeEvdData';
// import KNIFE_TASK from './knifeTask';
// import KnifeAssembly from './Knife_Assembly.json';
// import KnifeAssembly from './Knife_Assembly_Refactor.json';
import KnifeAssembly from './Knife_Assembly_Simple_VP.json';
import { STATUS } from './Constants';
import { node } from 'webpack';
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

const useStore = create(computed(immer(store),computedSlice));

// useStore.subscribe(store=>Object.values(store.programData).filter(n=>n.properties.status===STATUS.PENDING),(v)=>{console.log('REPLANNING',v);useStore.get().performPlanProcess()})

useStore.getState().setData(KnifeAssembly);
// useStore.getState().loadSolver();
// useStore.getState().setSolver()
console.log(useStore.getState())
// console.log(KNIFE_TASK.environment.trajectories[0])
useStore.getState().setUrl('ws://localhost:9090');

export default useStore;