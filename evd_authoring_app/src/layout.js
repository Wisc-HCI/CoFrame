
// All sizes in pixels
const componentDesiredSizes = {
    programEditor: {
        body: {
            height: null,
            width: 650,
        },
        header: {
            height: 55,
            width: null
        }
    },
    header: {
        height: 55,
        width: null
    },
    checklist: {
        body: {
            height: null,
            width: 350
        },
        header: {
            height: 90,
            width: null
        }
    },
    simulator: {
        header: {
            height: 55,
            width: null,
        },
        body: {
            height: null,
            width: null
        }
    }
};


const computeHeaderLayout = (layoutObj) => {

    // handle header layout
    layoutObj.header.height = componentDesiredSizes.header.height;
    if (layoutObj.header.height > layoutObj.totalHeight) {
        layoutObj.header.height = layoutObj.totalHeight;
    }
    layoutObj.distanceFromTop += layoutObj.header.height;
    layoutObj.remainingHeight -= layoutObj.header.height;

    layoutObj.header.width = layoutObj.totalWidth;

    layoutObj.body.height = layoutObj.remainingHeight;
    layoutObj.body.width = layoutObj.totalWidth;

    return layoutObj;
}


const computeMainLayout = (layoutObj) => {

    layoutObj.remainingWidth = layoutObj.totalWidth;

    // handle program layout
    layoutObj.body.program.height = layoutObj.remainingHeight;
    layoutObj.body.program.width = componentDesiredSizes.programEditor.body.width;
    if (layoutObj.body.program.width > layoutObj.remainingWidth) {
        layoutObj.body.program.width = layoutObj.remainingWidth;
    }
    layoutObj.remainingWidth -= layoutObj.body.program.width;

    layoutObj.body.program.header.height = componentDesiredSizes.programEditor.header.height;
    layoutObj.body.program.header.width = layoutObj.body.program.width;

    layoutObj.body.program.body.width = layoutObj.body.program.width;
    layoutObj.body.program.body.height = layoutObj.body.program.height - layoutObj.body.program.header.height;


    // handle checklist layout
    layoutObj.body.checklist.height = layoutObj.remainingHeight;
    layoutObj.body.checklist.width = componentDesiredSizes.checklist.body.width;
    if (layoutObj.body.checklist.width > layoutObj.remainingWidth) {
        layoutObj.body.checklist.width = layoutObj.remainingWidth;
    }
    layoutObj.remainingWidth -= layoutObj.body.checklist.width;

    layoutObj.body.checklist.header.height = componentDesiredSizes.checklist.header.height;
    layoutObj.body.checklist.header.width = layoutObj.body.checklist.width;

    layoutObj.body.checklist.body.width = layoutObj.body.checklist.width;
    layoutObj.body.checklist.body.height = layoutObj.body.checklist.height - layoutObj.body.checklist.header.height;

    // handle simulator layout
    layoutObj.body.simulator.width = layoutObj.remainingWidth;
    layoutObj.body.simulator.height = layoutObj.remainingHeight;

    layoutObj.body.simulator.header.height = componentDesiredSizes.simulator.header.height;
    layoutObj.body.simulator.header.width = layoutObj.body.simulator.width;

    layoutObj.body.simulator.body.height = layoutObj.body.simulator.height - layoutObj.body.simulator.header.height;
    layoutObj.body.simulator.body.width = layoutObj.body.simulator.width;

    return layoutObj;
}


export const computeLayout = (width, height) => {

    let layoutObj = {
        distanceFromTop: 0,
        remainingHeight: height,
        remainingWidth: width,
        totalHeight: height,
        totalWidth: width,

        header: {
            height: 0,
            width: 0
        },
        body: {
            height: 0,
            width: 0,
            program: {
                height: 0,
                width: 0,
                header: {
                    height: 0,
                    width: 0
                },
                body: {
                    height: 0,
                    width: 0
                }
            },
            simulator: {
                height: 0,
                width: 0,
                header: {
                    height: 0,
                    width: 0
                },
                body: {
                    height: 0,
                    width: 0
                }
            },
            checklist: {
                height: 0,
                width: 0,
                header: {
                    height: 0,
                    width: 0
                },
                body: {
                    height: 0,
                    width: 0
                }
            }
        }
    };

    layoutObj = computeHeaderLayout(layoutObj);
    layoutObj = computeMainLayout(layoutObj);

    return layoutObj;
}