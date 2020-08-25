import React, { Component } from 'react'

import ParametersPanel from './ParametersPanel';
import ChecklistPanel from './ChecklistPanel';

import { Stack } from 'office-ui-fabric-react';
import { CommandBar } from 'office-ui-fabric-react/lib/CommandBar';

export default class DetailPanel extends Component {

    constructor(props) {
        super(props);

        this.state = {
            activePanel: 'parameters'
        };
    }

    static getDesiredHeight() {
        return null;
    }

    static getDesiredWidth() {
        return 450; //px
    }

    render() {

        const padding = 15;
        const buttonHeight = 40;

        let width = this.props.width - (2 * padding);
        let height = this.props.height - (2 * padding);
        
        let distanceFromTop = padding + buttonHeight + this.props.distanceFromTop;

        const _items = [
            {
                key: 'Parameters',
                text: 'Parameters',
                iconProps: { iconName: 'AddNotesIcon' },
                onClick: () => console.log('Parameters'),
            },
            {
                key: 'Checklist',
                text: 'Checklist',
                iconProps: { iconName: 'AccountActivityIcon' },
                onClick: () => console.log('Checklist'),
            },
        ];

        let activePanel = null;
        if (this.state.activePanel === "parameters") {
            activePanel = (<ParametersPanel width={width} height={height} distanceFromTop={distanceFromTop} />);

        } else if (this.state.activePanel === "checklist") {
            activePanel = (<ChecklistPanel width={width} height={height} distanceFromTop={distanceFromTop} />);

        } else {
            activePanel = (<p>Invalid panel type</p>);

        }

        return (
            <div style={{ padding: `${padding}px`, width: `${this.props.width}px`, height: `${this.props.height}px`}}>
                <div style={{backgroundColor: this.props.theme.semanticColors.bodyBackground, boxShadow: "3px 3px 3px #000"}}>
                    <Stack>
                        <CommandBar 
                            items={_items}
                            ariaLabel="Use left and right arrow keys to navigate between commands"
                        />
                        {activePanel}
                    </Stack>
                </div>
            </div>
        );
    }
}