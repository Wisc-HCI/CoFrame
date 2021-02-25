import React, { Component } from 'react';
import { Stack } from 'office-ui-fabric-react';

import { Tile } from './Tile';
import { Simulator } from './Simulator';
import { ProgramEditor } from './ProgramEditor';
import { FrameButtons } from './FrameButtons';
import { ExpertChecklist } from './ExpertChecklist';

export class Body extends Component {

    constructor(props) {
        super(props);

        this.state = {
            frame: 'safety',
        };

        this.onFrameButtonClicked = this.onFrameButtonClicked.bind(this);
    }

    onFrameButtonClicked(frame) {
        this.setState({ frame });
    }

    render() {
        const { frame } = this.state;
        const { layoutObj, theme, mainPadding } = this.props;

        return (
            <div
                style={{
                    width: `${layoutObj.mainWidth}px`,
                    height: `${layoutObj.mainHeight}px`,
                }}
            >
                <Stack horizontal>

                    <div style={{
                            paddingRight: `${mainPadding / 2}px`,
                            paddingTop: `${mainPadding}px`,
                            paddingBottom: `${mainPadding}px`,
                        }}
                    >
                        <Tile
                            theme={theme}
                            width={layoutObj.checklistWidth - mainPadding / 2}
                            height={layoutObj.checklistHeight - mainPadding}
                        >
                            <FrameButtons frame={frame} callback={this.onFrameButtonClicked} />
                            <ExpertChecklist frame={frame} />
                        </Tile>
                    </div>

                    <div
                        style={{
                            paddingLeft: `${mainPadding / 2}px`,
                            paddingRight: `${mainPadding / 2}px`,
                            paddingTop: `${mainPadding}px`,
                            paddingBottom: `${mainPadding}px`,
                        }}
                    >
                        <Tile
                            theme={theme}
                            width={layoutObj.simulatorWidth - mainPadding}
                            height={layoutObj.simulatorHeight - mainPadding}
                        >
                            <Simulator frame={frame} />
                        </Tile>     
                    </div>

                    <div
                        style={{
                            paddingTop: `${mainPadding}px`,
                            paddingBottom: `${mainPadding}px`,
                            paddingLeft: `${mainPadding / 2}px`,
                        }}
                    >
                        <Tile 
                            theme={theme}
                            width={layoutObj.progamWidth - mainPadding / 2}
                            height={layoutObj.programHeight - mainPadding}
                        >
                            <ProgramEditor />
                        </Tile>  
                    </div>

                    
                </Stack>
            </div>
        );
    }
    
}