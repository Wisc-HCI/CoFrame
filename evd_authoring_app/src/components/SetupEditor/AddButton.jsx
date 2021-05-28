import React from 'react';

import { useId } from '@fluentui/react-hooks';
import {Button,Tooltip} from 'antd';


export const AddButton = (props) => {

    const {
        type,
        callback,
        disabled
    } = props;

    const tooltipId = useId('add-tooltip');

    return (
        <Tooltip
            title={`Add a new ${type}`}
            id={tooltipId}
            calloutProps={{ gapSpace: 0 }}
            styles={{

                    display: 'inline-block',
                    width: '90%',
                    boxShadow: '3px 3px 3px #000'

            }}
        >
            <Button
                type="primary"

                styles={{

                        width: '90%'

                }}
                onClick={callback}
                disabled={disabled}
            >
                 {`Add New ${type}`}
            </Button>
        </Tooltip>

    );
};
