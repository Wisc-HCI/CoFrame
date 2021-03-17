import React from 'react';

import { Stack } from 'office-ui-fabric-react';

import { ChecklistTile } from './ChecklistTile';
import { SimulatorTile } from './SimulatorTile';
import { ProgramTile } from './ProgramTile';

import { FrameContext } from "../../contexts";


export class Body extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            frame: 'safety',
        };

        this.onFrameChangeCallback = this.onFrameChangeCallback.bind(this);
    }

    onFrameChangeCallback(frame) {
        this.setState({ frame });
    }

    render() {
        const { frame } = this.state;
        
        const { 
            layoutObj, 
            mainPadding 
        } = this.props;

        return (
            <FrameContext.Provider
                value={{
                    frame: frame,
                    changeFrame: this.onFrameChangeCallback
                }}
            >
                <div
                    style={{
                        width: `${layoutObj.body.width}px`,
                        height: `${layoutObj.body.height}px`,
                    }}
                >
                    <Stack horizontal>

                        <ChecklistTile
                            mainPadding={mainPadding}
                            layoutChecklist={layoutObj.body.checklist}
                        />

                        <SimulatorTile 
                            mainPadding={mainPadding}
                            layoutSimulator={layoutObj.body.simulator}
                        />

                        <ProgramTile
                            mainPadding={mainPadding}
                            layoutProgram={layoutObj.body.program}                
                        />

                    </Stack>
                </div>
            </FrameContext.Provider>
            
        );
    }
    
}