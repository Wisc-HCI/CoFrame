import {omit, mapValues} from "lodash";
import { DATA_TYPES } from "simple-vp";
import { instanceTemplateFromSpec } from "simple-vp";

export const ProgramStoreSlice = (set, get) => ({
  currentProgramId: '',
  allPrograms: {},
  addProgramData: (id, data) => set(state => {
    state.allPrograms[id] = data;
  }),
  deleteProgramData: (id) => set(state => {
    delete state.allPrograms[id];
  }),
  setCurrentProgram: (id) => set(state => {
    if (id in state.allPrograms) {
      // Store/update program changes
      if (state.currentProgramId !== "") {
        state.allPrograms[state.currentProgramId] = {...state.tabs, ...state.activeTab, ...state.programData}
      }

      // Set new program id
      state.currentProgramId = id;

      // Copied from the setdata function
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
      state.programData = newData;
      state.loaded = true;
      if (tabs) {
        state.tabs = tabs
      }
      if (activeTab) {
        state.activeTab = activeTab
      }
    }
  }),
});
