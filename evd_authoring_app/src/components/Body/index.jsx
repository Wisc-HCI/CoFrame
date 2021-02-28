import React, { Component } from 'react';
import { Stack } from 'office-ui-fabric-react';

import { ChecklistTile } from './ChecklistTile';
import { SimulatorTile } from './SimulatorTile';
import { ProgramTile } from './ProgramTile';


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

        console.log(layoutObj);

        return (
            <div
                style={{
                    width: `${layoutObj.body.width}px`,
                    height: `${layoutObj.body.height}px`,
                }}
            >
                <Stack horizontal>

                    <ChecklistTile
                        frame={frame}
                        mainPadding={mainPadding}
                        theme={theme}
                        layoutChecklist={layoutObj.body.checklist}
                        onFrameButtonCallback={this.onFrameButtonClicked}
                    />

                    <SimulatorTile 
                        frame={frame}
                        mainPadding={mainPadding}
                        theme={theme}
                        layoutSimulator={layoutObj.body.simulator}
                    />

                    <ProgramTile
                        mainPadding={mainPadding}
                        theme={theme}
                        layoutProgram={layoutObj.body.program}                
                    />

                </Stack>
            </div>
        );
    }
    
}