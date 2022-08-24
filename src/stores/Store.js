import create from 'zustand';
import shallow from 'zustand/shallow';
import { subscribeWithSelector } from 'zustand/middleware';
import { immer } from 'zustand/middleware/immer'
import { computed } from 'zustand-middleware-computed-state';
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
import {performCompileProcess} from './planner-worker'

// const immer = (config) => (set, get, api) =>
//   config(
//     (partial, replace) => {
//       const nextState =
//         typeof partial === "function" ? produce(partial) : partial;
//       return set(nextState, replace);
//     },
//     get,
//     api
// );

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

useStore.subscribe(state=>
  lodash.mapValues(state.programData,(value)=>{
    return value?.properties?.status ? value.properties.status : STATUS.PENDING
  }),
  (currentStatuses,previousStatuses)=>{
    if (Object.keys(currentStatuses).some(id=>currentStatuses[id]===STATUS.PENDING && previousStatuses[id]!==STATUS.PENDING)) {
      console.log("REPLANNING")
      useStore.getState().performCompileProcess()
    }
    
    // const data = useStore.getState();
    // console.log(data);
    // performCompileProcess({programData:data.programData,objectTypes:data.programSpec.objectTypes})
  },
  {equalityFn:shallow}
)

if (Object.keys(useStore.getState().programData).length === 0) {
  useStore.getState().setData(KnifeAssembly);
}

export default useStore;