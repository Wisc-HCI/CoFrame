// import create from 'zustand'
import create from 'zustand';
import shallow from 'zustand/shallow'
import { subscribeWithSelector, persist } from 'zustand/middleware'
import { computed } from 'zustand-middleware-computed-state'
import produce from "immer";
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
import { SceneSlice } from 'robot-scene';
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
//import { useYDoc } from 'zustand-yjs'
import * as Y from "yjs";
import { WebrtcProvider } from "y-webrtc";
import yjs from 'zustand-middleware-yjs';


window.localStorage.setItem("log", "y-webrtc");

const doc = new Y.Doc();
new WebrtcProvider("CoFrame-AR", doc);

// const connectDoc = (doc) => {
//   console.log('connect to a provider with room', doc.guid)
//   return () => console.log('disconnect', doc.guid)
// }

// const yDoc = useYDoc('myDocGuid', connectDoc);

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
    ...SceneSlice(set,get),
    ...ProgrammingSlice(set,get), // default programming slice for simple-vp
    ...ProgrammingSliceOverride(set,get), // overrides data-editing functionality to update pending properties
    ...GuiSlice(set,get),
    ...ReviewSlice(set,get),
    ...EvdSlice(set,get),
    ...RosSlice(set,get),
})

//const useStore = create(subscribeWithSelector(computed(immer(store),computedSlice)));

const useStore = create(
  yjs(doc,"shared",
  subscribeWithSelector(
    computed(
      immer(store),computedSlice)
      )
    ));

console.log("getState: ", useStore.getState());

//console.log("tempStore:", tempStore.getState());


useStore.getState().setData(KnifeAssembly);
//tempStore.getState().setData(KnifeAssembly);

useStore.subscribe(store=>
  lodash.mapValues(store.programData,(value)=>{
    return value?.properties?.status ? value.properties.status : STATUS.PENDING
  }),
  ()=>{
    console.log("REPLANNING")
    useStore.getState().performCompileProcess()
  },
  {equalityFn:shallow}
)

useStore.getState().performCompileProcess()

// useStore.getState().loadSolver();
// useStore.getState().setSolver()
console.log(useStore.getState())
// console.log(KNIFE_TASK.environment.trajectories[0])
useStore.getState().setUrl('ws://localhost:9090');

export default useStore;