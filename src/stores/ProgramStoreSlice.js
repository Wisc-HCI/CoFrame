import {omit, mapValues} from "lodash";
import { DATA_TYPES } from "simple-vp";
import { instanceTemplateFromSpec } from "simple-vp";

export const ProgramStoreSlice = (set, get) => ({
  currentProgramData: {},
  currentProgramId: '',
  allPrograms: {},
  addProgramData: (id, data) => set(state => {
    state.allPrograms[id] = data;
  }),
  deleteProgramData: (id) => set(state => {
    delete state.allPrograms[id];
  }),
  setCurrentProgram: (id) => set(state => {
    // if (key in state.allPrograms) {
      state.currentProgramData = state.allPrograms[id];
      state.currentProgramId = id;
      const {tabs, activeTab, ...programData} = state.allPrograms[id];
      const newData = mapValues(programData, (d) => {
        if (d.dataType === DATA_TYPES.INSTANCE) {
          const defaultv = instanceTemplateFromSpec(
            d.type,
            state.programSpec.objectTypes[d.type],
            false
          );
          return {
            ...d,
            properties: {
              ...defaultv.properties,
              ...omit(d.properties, [
                "status",
                "compiled",
                "compileFn",
                "updateFields",
              ]),
            },
          };
        } else {
          return d;
        }
      });
      // console.log(newData);
      state.programData = newData;
      state.loaded = true;
      if (tabs) {
        state.tabs = tabs
      }
      if (activeTab) {
        state.activeTab = activeTab
      }
    // }
  }),
});
