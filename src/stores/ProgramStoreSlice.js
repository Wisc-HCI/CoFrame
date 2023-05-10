export const ProgramStoreSlice = (set, get) => ({
  currentProgramId: '',
  allPrograms: {},
  allCompiledData: {},
  addProgramData: (id, data, compiledData) => set(state => {
    state.allPrograms[id] = data;
    state.allCompiledData[id] = compiledData;;
  }),
  deleteProgramData: (id) => set(state => {
    delete state.allPrograms[id];
  }),
  updateAllCompiledData: (id, compiledData) => set(state => {
    state.allCompiledData[id] = compiledData;
  }),
  updateProgramData: (id, data) => set(state => {
    state.allPrograms[id] = data;
  }),
});
