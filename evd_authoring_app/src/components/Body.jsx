import React from 'react';

import { Stack } from 'office-ui-fabric-react';

import Simulator from './components/Simulator';
import ProgramEditor from './components/Blockly/ProgramEditor';

function Body(props) {

    const { layoutObj, theme, mainPadding } = props;

    return (
        <div
            style={{
                width: `${layoutObj.mainWidth}px`,
                height: `${layoutObj.mainHeight}px`,
            }}
        >
            <Stack horizontal>
                <div
                style={{
                    paddingTop: `${mainPadding}px`,
                    paddingBottom: `${mainPadding}px`,
                    paddingRight: `${mainPadding / 2}px`,
                }}
                >
                <ProgramEditor
                    theme={theme}
                    width={layoutObj.progamWidth - mainPadding / 2}
                    height={layoutObj.programHeight - mainPadding}
                />
                </div>

                <div
                style={{
                    paddingLeft: `${mainPadding / 2}px`,
                    paddingTop: `${mainPadding}px`,
                    paddingBottom: `${mainPadding}px`,
                }}
                >
                <Simulator
                    theme={theme}
                    width={layoutObj.simulatorWidth - mainPadding / 2}
                    height={layoutObj.simulatorHeight - mainPadding}
                />
                </div>
            </Stack>
        </div>
    );
}

export default Body;