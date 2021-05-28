import React from 'react';

import { TooltipHost } from '@fluentui/react/lib/Tooltip';
import { IconButton } from '@fluentui/react/lib/Button';
import { useId } from '@fluentui/react-hooks';
import {Button,Tooltip} from 'antd';
import { DeleteOutlined } from '@ant-design/icons';



export const DeleteButton = (props) => {

    const {
        type,
        callback,
        disabled
    } = props;

    const tooltipId = useId('delete-tooltip');

    return (
        <Tooltip
            title={(disabled) ? `Cannot Delete ${type}` : `Delete ${type}`}
            id={tooltipId}
            calloutProps={{ gapSpace: 0 }}
        >
            <Button
                type = 'primary'

                icon={ <DeleteOutlined />}

                onClick={callback}
                disabled={disabled}
            >
            Delete

            </Button>
        </Tooltip>
    );
};
