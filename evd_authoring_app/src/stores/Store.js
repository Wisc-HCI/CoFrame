// import create from 'zustand'
import create from 'zustand-store-addons';
import produce from "immer";
import { persist } from "zustand/middleware"
import {GuiSlice} from './GuiSlice';
import {ReviewSlice} from './ReviewSlice';
import {EvdSlice} from './EvdSlice';
import {RosSlice} from './RosSlice';
// import {SceneSlice} from 'robot-scene';
import {ComputedSlice} from './ComputedSlice';
// import {WatchedSlice} from './WatchedSlice';
// import { computed } from 'zustand-middleware-computed-state';
// import {SimSlice} from './SimSlice';

// import { INITIAL_SIM } from "./initialSim";
// import fakeEvdData from './fakeEvdData';
// import KNIFE_TASK from './knifeTask';
import KnifeAssembly from './Knife_Assembly.json';

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
    ...GuiSlice(set,get),
    ...ReviewSlice(set,get),
    ...EvdSlice(set,get),
    ...RosSlice(set,get),
})

// const useStore = create(store,{...ComputedSlice,middleware:[immer,persist]});
const useStore = create(store,{...ComputedSlice,middleware:[immer]});

// useStore.getState().setProgram(fakeEvdData.arbitrary.program);
useStore.getState().setProgram(KnifeAssembly)
// console.log(KNIFE_TASK.environment.trajectories[0])
useStore.getState().setUrl('ws://localhost:9090');

export default useStore;