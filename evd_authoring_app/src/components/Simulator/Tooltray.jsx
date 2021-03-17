import React from 'react';

import { ChoiceGroup } from 'office-ui-fabric-react/lib/ChoiceGroup';

import { ThemeContext } from "../../contexts";

import './index.css';


export const Tooltray = (props) => {

    const { 
        active, 
        callback 
    } = props;
    
    const options = [
        { key: 'rotate', text: 'Rotate' },
        { key: 'translate', text: 'Translate' }
    ];

    return (
        <ThemeContext.Consumer>
            { value => (
                <div 
                    style={{
                        padding: '5px 10px 0 10px',
                        background: value.theme.semanticColors.bodyBackground, 
                        boxShadow: '3px 3px 3px #000'
                    }}
                >
                    <i>Camera</i>
                    <ChoiceGroup
                        className="simulator-tooltray-choicegroup"
                        defaultSelectedKey={active}
                        options={options}
                        onChange={callback}
                    />
                </div>
            )}
        </ThemeContext.Consumer>
    );
};