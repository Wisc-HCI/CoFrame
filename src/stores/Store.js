import create from 'zustand';
import shallow from 'zustand/shallow'
import { persist, subscribeWithSelector } from 'zustand/middleware'
import { computed } from 'zustand-middleware-computed-state'
import produce from "immer";
import {GuiSlice} from './GuiSlice';
import {ReviewSlice} from './ReviewSlice';
import {EvdSlice} from './EvdSlice';
import {RosSlice} from './RosSlice';
import {ProgrammingSlice} from 'simple-vp';
import { ProgrammingSliceOverride } from './ProgrammingSlice';
import {computedSlice} from './ComputedSlice';
import { SceneSlice } from 'robot-scene';
import lodash from 'lodash';
import KnifeAssembly from './Knife_Assembly_Simple_VP.json';
import PandaDemo from './Panda_Demo.json'
import { STATUS } from './Constants';

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

const immerStore = immer(store);
const computedStore = computed(immerStore,computedSlice);
const subscribeStore = subscribeWithSelector(computedStore);

const useStore = create(subscribeStore);
// const useSyncStore = create(yjs(doc,"shared",subscribeStore))

// console.log("getState: ", useStore.getState());

useStore.subscribe(state=>
  lodash.mapValues(state.programData,(value)=>{
    return value?.properties?.status ? value.properties.status : STATUS.PENDING
  }),
  ()=>{
    console.log("REPLANNING")
    useStore.getState().performCompileProcess()
  },
  {equalityFn:shallow}
)

if (Object.keys(useStore.getState().programData).length === 0) {
  useStore.getState().setData(KnifeAssembly);
}

export default useStore;