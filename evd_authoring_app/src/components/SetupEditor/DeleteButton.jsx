import React from 'react';

import { TooltipHost } from '@fluentui/react/lib/Tooltip';
import { IconButton } from '@fluentui/react/lib/Button';
import { useId } from '@fluentui/react-hooks';


export const DeleteButton = (props) => {

    const {
        type,
        callback,
        disabled
    } = props;

    const tooltipId = useId('delete-tooltip');

    return (
        <TooltipHost
            content={(disabled) ? `Cannot Delete ${type}` : `Delete ${type}`}
            id={tooltipId}
            calloutProps={{ gapSpace: 0 }}
            styles={{
                root: { display: 'inline-block', textAlign: 'right' }
            }}
        >
            <IconButton 
                primary={true}
                iconProps={{ iconName: 'Delete' }}
                styles={{
                    root: {
                        fontSize: '50px', textAlign: 'right' 
                    }
                }}
                onClick={callback}
                disabled={disabled}
            /> 
        </TooltipHost>
    );
};