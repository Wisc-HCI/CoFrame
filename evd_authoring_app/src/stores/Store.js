import create from 'zustand'
import produce from "immer";
import {GuiSlice} from './GuiSlice';
import {ReviewSlice} from './ReviewSlice';
import {EvdSlice} from './EvdSlice';
import {RosSlice} from './RosSlice';
// import {SimSlice} from './SimSlice';

import { INITIAL_SIM } from "./initialSim";
import fakeEvdData from './fakeEvdData';


const immer = (config) => (set, get, api) =>
  config((fn) => set(produce(fn)), get, api);

const store = (set, get) => ({
    ...GuiSlice(set, get),
    ...ReviewSlice(set, get),
    ...EvdSlice(set, get),
    ...RosSlice(set, get)
    // ...SimSlice(set, get)
})

const useStore = create(immer(store));

useStore.getState().setProgram(fakeEvdData.arbitrary.program);
useStore.getState().setUrl('ws://localhost:9090');
useStore.getState().setup(INITIAL_SIM);

export default useStore;