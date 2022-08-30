import create from 'zustand';
import { subscribeWithSelector, persist } from 'zustand/middleware';
import { TauriStorage } from './TauriStorage';

const compiledStore = (set,get) => ({});

const persistStore = persist(compiledStore, {
    name: "coframe-compiled-store",
    getStorage: ()=>TauriStorage
  });

const subscribeStore = subscribeWithSelector(persistStore);



const useCompiledStore = create(subscribeStore);

export default useCompiledStore;