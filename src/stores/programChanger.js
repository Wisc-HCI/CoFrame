import useCompiledStore from "./CompiledStore";
import useStore from "./Store";

export const swapProgram = (id) => {
    // Verify program ID exists
    let programKeys = Object.keys(useStore.getState().allPrograms);
    if (programKeys.includes(id)) {
    let currentProgramId = useStore.getState().currentProgramId;
    let newCompiledData = useStore.getState().allCompiledData[id];
    let newProgramData = useStore.getState().allPrograms[id];
    let updateAllCompiledData = useStore.getState().updateAllCompiledData;

    if (currentProgramId !== "") {
        // Store current program
        let updateProgramData = useStore.getState().updateProgramData;
        let currentTabs = useStore.getState().tabs;
        let currentActiveTab = useStore.getState().activeTab;
        let currentProgramData = useStore.getState().programData;
        updateProgramData(currentProgramId, {...currentTabs, ...currentActiveTab, ...currentProgramData});

        // Store the compiled data
        updateAllCompiledData(currentProgramId, useCompiledStore.getState());

        // Temporarily "blank" the state
        useStore.getState().setData({});
    }

    // Load the new compiled data
    useCompiledStore.setState(newCompiledData);

    // Load the new program
    useStore.getState().setData(newProgramData);
    }
};
