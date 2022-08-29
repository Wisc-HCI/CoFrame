import create from 'zustand';
import { subscribeWithSelector } from 'zustand/middleware';

const compiledStore = (set,get) => ({});

const subscribeStore = subscribeWithSelector(compiledStore);

const useCompiledStore = create(subscribeStore);

export default useCompiledStore;