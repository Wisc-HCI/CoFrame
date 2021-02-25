
// All sizes in pixels
const componentDesiredSizes = {
    programEditor: {
        height: null,
        width: 600
    },
    header: {
        height: 55,
        width: null
    },
    checklist: {
        height: null,
        width: 400
    }
};


const computeHeaderLayout = (layoutObj) => {
    const newLayoutObj = { ...layoutObj };

    // handle header layout
    newLayoutObj.headerHeight = componentDesiredSizes.header.height;
    if (newLayoutObj.headerHeight > newLayoutObj.totalHeight) {
        newLayoutObj.headerHeight = newLayoutObj.totalHeight;
    }
    newLayoutObj.distanceFromTop += newLayoutObj.headerHeight;
    newLayoutObj.remainingHeight -= newLayoutObj.headerHeight;

    newLayoutObj.headerWidth = newLayoutObj.totalWidth;

    newLayoutObj.mainHeight = newLayoutObj.remainingHeight;
    newLayoutObj.mainWidth = newLayoutObj.totalWidth;

    return newLayoutObj;
}


const computeMainLayout = (layoutObj) => {
    const newLayoutObj = { ...layoutObj };

    let remainingWidth = newLayoutObj.totalWidth;

    // handle program layout
    newLayoutObj.programHeight = newLayoutObj.remainingHeight;
    newLayoutObj.progamWidth = componentDesiredSizes.programEditor.width;
    if (newLayoutObj.progamWidth > remainingWidth) {
        newLayoutObj.progamWidth = remainingWidth;
    }
    remainingWidth -= newLayoutObj.progamWidth;

    // handle checklist layout
    newLayoutObj.checklistHeight = newLayoutObj.remainingHeight;
    newLayoutObj.checklistWidth = componentDesiredSizes.checklist.width;
    if (newLayoutObj.checklistWidth > remainingWidth) {
        newLayoutObj.checklistWidth = remainingWidth;
    }
    remainingWidth -= newLayoutObj.checklistWidth;

    // handle simulator layout
    newLayoutObj.simulatorWidth = remainingWidth;
    newLayoutObj.simulatorHeight = newLayoutObj.remainingHeight;

    return newLayoutObj;
}


export const computeLayout = (width, height) => {

    let layoutObj = {
        distanceFromTop: 0,
        remainingHeight: height,
        totalHeight: height,
        totalWidth: width,
        headerHeight: 0,
        headerWidth: 0,
        mainHeight: 0,
        mainWidth: 0,
        programHeight: 0,
        programWidth: 0,
        simulatorHeight: 0,
        simulatorWidth: 0,
        checklistHeight: 0,
        checklistWidth: 0
    };

    layoutObj = computeHeaderLayout(layoutObj);
    layoutObj = computeMainLayout(layoutObj);

    return layoutObj;
}