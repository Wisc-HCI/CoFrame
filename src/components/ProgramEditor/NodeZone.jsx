import React from 'react';
// import { Row } from 'antd';
// import { DeleteFilled } from '@ant-design/icons';
import { useDroppable } from '@dnd-kit/core';
import { Block } from './Blocks';
import useStore from '../../stores/Store';

export const NodeZone = ({parentId, field, ancestors, uuid, context, dragDisabled, dropDisabled}) => {

    const {dropRef} = useDroppable({
        id:JSON.stringify({parentId,field}),
        data: {uuid: parentId, field: field},
        disabled: uuid !== undefined || dropDisabled
    });
    const dragItem = useStore(state=>state.dragItem);
    const dropItem = useStore(state=>state.dropItem);

    const validHover = dropItem?.data?.uuid === parentId && dropItem?.data?.field === field && ancestors[0].accepts.includes(dragItem?.data?.type);

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
        <div className='nodrag' ref={dropRef} style={styles} >
            {validHover ? (
                <Block ancestors={dragItem.ancestors} staticData={dragItem.data} context={dragItem.context} dragDisabled={true}/>
            ) : uuid ? (
                <Block ancestors={ancestors} uuid={uuid} context={context} dragDisabled={dragDisabled}/>
            ) : (
                <span>No Value</span>
            )}
        </div>
        
    )
};

