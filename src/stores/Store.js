// import create from 'zustand'
import create from 'zustand';
import { computed } from 'zustand-middleware-computed-state'
import produce from "immer";
import { persist } from "zustand/middleware";
import {GuiSlice} from './GuiSlice';
import {ReviewSlice} from './ReviewSlice';
import {EvdSlice} from './EvdSlice';
import {RosSlice} from './RosSlice';
import {ProgrammingSlice} from 'simple-vp';
// import {SceneSlice} from 'robot-scene';
import {computedSlice} from './ComputedSlice';
// import {WatchedSlice} from './WatchedSlice';
// import { computed } from 'zustand-middleware-computed-state';
// import {SimSlice} from './SimSlice';

// import { INITIAL_SIM } from "./initialSim";
// import fakeEvdData from './fakeEvdData';
// import KNIFE_TASK from './knifeTask';
// import KnifeAssembly from './Knife_Assembly.json';
import KnifeAssembly from './Knife_Assembly_Refactor.json';
// import { Solver } from '@people_and_robots/lively_tk';
// import {ur3e} from './ur3e.xml';
// import {buffer} from "@people_and_robots/lively_tk";
// import {greet} from './hello_wasm'
import { primitiveTypes } from './templates';

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
    ...ProgrammingSlice(set,get),
    ...ReviewSlice(set,get),
    ...EvdSlice(set,get),
    ...RosSlice(set,get),
})

// const useStore = create(store,{...computedSlice,middleware:[immer,persist]});
const useStore = create(computed(immer(store),computedSlice));

// async function getSolver() {
//   console.log(buffer);
//   const module = await WebAssembly.compile(buffer);
//   console.log(module);
//   const instance = await WebAssembly.instantiate(module,{});
//   console.log(instance);
//   instance.exports.greet();
//   // const solver = instance.exports.Solver(urdf,[]);
//   // console.log(solver)
// }

// // getSolver()
// async function doGreet() {
//   const promise = await import('./hello_wasm');
//   promise.greet();
// }
// doGreet();

useStore.getState().setData(KnifeAssembly);
// useStore.getState().loadSolver();
// useStore.getState().setSolver()
console.log(useStore.getState())
// console.log(KNIFE_TASK.environment.trajectories[0])
useStore.getState().setUrl('ws://localhost:9090');

export default useStore;