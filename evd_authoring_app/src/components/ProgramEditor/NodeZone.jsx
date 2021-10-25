import React from 'react';
import { Row } from 'antd';
import { DeleteFilled } from '@ant-design/icons';
import { useDroppable } from '@dnd-kit/core';
import { Block } from './Blocks';

export const NodeZone = ({parentId, field, ancestors, uuid, context, dragDisabled}) => {

    const {dropRef} = useDroppable({id:JSON.stringify({parentId,field})});

    const styles = {
        backgroundColor: 'rgba(0,0,0,0.5)',
        borderRadius: 5,
        minWidth: 70,
        minHeight: 30,
        padding: 5,
        textAlign: 'center',
        fontSize: 14
    }

    return (
        <div ref={dropRef} style={styles} >
            {uuid && <Block ancestors={ancestors} uuid={uuid} context={context} dragDisabled={dragDisabled}/>}
        </div>
        
    )
};

