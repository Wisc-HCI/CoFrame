import React, { Component } from 'react';

import { Stack } from '@fluentui/react/lib/Stack';

import { ChecklistTile } from './ChecklistTile';
import { SimulatorTile } from './SimulatorTile';
import { ProgramTile } from './ProgramTile';

import { 
    FrameContext, 
    ControlsContext 
} from "../../contexts";


export class Body extends Component {

    constructor(props) {
        super(props);

        this.state = {
            frame: 'safety',
            controlsInSetup: true,
            controlsChecklistItem: null,
            controlsSetupItem: null
        };

        this.onFrameChangeCallback = this.onFrameChangeCallback.bind(this);
        this.onSetupStateChangeCallback = this.onSetupStateChangeCallback.bind(this);
        this.onChecklistItemChangeCallback = this.onChecklistItemChangeCallback.bind(this);
        this.onSetupItemChangeCallback = this.onSetupItemChangeCallback.bind(this);
    }

    onFrameChangeCallback(frame) {
        this.setState({ frame });
    }

    onSetupStateChangeCallback(val) {
        this.setState({ controlsInSetup: val });
    }

    onChecklistItemChangeCallback(item) {
        this.setState({ controlsChecklistItem: item });
    }

    onSetupItemChangeCallback(item) {
        this.setState({ controlsSetupItem: item })
    }

    render() {
        
        const { 
            frame, 
            controlsInSetup, 
            controlsChecklistItem,
            controlsSetupItem
        } = this.state;
        
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
                <ControlsContext.Provider
                    value={{
                        inSetup: controlsInSetup,
                        checklistItem: controlsChecklistItem,
                        setupItem: controlsSetupItem,
                        changeSetupState: this.onSetupStateChangeCallback,
                        changeChecklistItem: this.onChecklistItemChangeCallback,
                        changeSetupItem: this.onSetupItemChangeCallback
                    }}
                >
                    
                    <div
                        style={{
                            width: `${layoutObj.body.width}px`,
                            height: `${layoutObj.body.height}px`,
                        }}
                    >
                        <Stack horizontal>

                            {layoutObj.body.checklist.display 
                                ? (
                                    <ChecklistTile
                                        mainPadding={mainPadding}
                                        layoutChecklist={layoutObj.body.checklist}
                                    />
                                ) 
                                : undefined 
                            }

                            <SimulatorTile 
                                mainPadding={mainPadding}
                                layoutSimulator={layoutObj.body.simulator}
                                checklistExists={layoutObj.body.checklist.display}
                            />

                            <ProgramTile
                                mainPadding={mainPadding}
                                layoutProgram={layoutObj.body.program}                
                            />

                        </Stack>
                    </div>

                </ControlsContext.Provider> 
            </FrameContext.Provider>
        );
    }
    
}