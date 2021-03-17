import React from 'react';

import { 
    IconButton,
    TooltipHost
} from 'office-ui-fabric-react';
import { useId } from '@uifabric/react-hooks';


export const AddButton = (props) => {

    const { 
        type, 
        callback 
    } = props;

    const tooltipId = useId('tooltip');

    return (
        <TooltipHost
            content={`Add a new ${type}`}
            id={tooltipId}
            calloutProps={{ gapSpace: 0 }}
            styles={{
                root: { display: 'inline-block' }
            }}
        >
            <IconButton 
                primary={true}
                title={`Add ${type}`}
                iconProps={{ iconName: 'CirclePlus' }}
                styles={{
                    root: {
                        fontSize: '50px'
                    }
                }}
                onClick={callback}
            /> 
        </TooltipHost>
    );
};