import React from 'react';

import { PrimaryButton } from '@fluentui/react/lib/Button';
import { TooltipHost } from '@fluentui/react/lib/Tooltip';
import { useId } from '@fluentui/react-hooks';

import { ThemeContext } from "../../contexts";


export const AddButton = (props) => {

    const {
        type, 
        callback,
        disabled 
    } = props;

    const tooltipId = useId('add-tooltip');

    return (
        <ThemeContext.Consumer>
            { value => (
                
                <TooltipHost
                    content={`Add a new ${type}`}
                    id={tooltipId}
                    calloutProps={{ gapSpace: 0 }}
                    styles={{
                        root: {
                            backgroundColor: value.theme.semanticColors.bodyBackground,
                            display: 'inline-block',
                            width: '100%',
                            boxShadow: '3px 3px 3px #000'
                        }
                    }}
                >
                    <PrimaryButton
                        text={`Add New ${type}`}
                        styles={{
                            root: {
                                width: '100%'
                            }
                        }}
                        onClick={callback}
                        disabled={disabled}
                    /> 
                </TooltipHost>
                    
            )}
        </ThemeContext.Consumer> 
        
    );
};