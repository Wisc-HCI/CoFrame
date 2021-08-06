import React from 'react';
import { Tag } from 'antd';
import { useDrop } from 'react-dnd';
import blockStyles from './blockStyles';

export const ParameterZone = ({itemType,displayText,acceptTypes,onDrop,onRemove,canRemove}) => {

    const [{isOver, canDrop}, drop] = useDrop({
        accept: acceptTypes,
        drop: (item, _) => {
            console.log(item)
            onDrop(item)
        },
        canDrop: (item, _) => (acceptTypes.indexOf(item.type)>=0 && canRemove),
        collect: (monitor) => ({
            isOver: monitor.isOver(),
            canDrop: monitor.canDrop()
        })
    })

    const containerStyle = {
        backgroundColor: 'rgba(0,0,0,0.5)',
        borderRadius: 5,
        minWidth: 20,
        minHeight:27,
        padding:3,
        textAlign:'center'
    }

    return (
        <div ref={drop} style={containerStyle} >
            {displayText && (!isOver || !canDrop) && (
            <Tag color={blockStyles[itemType]} closable={canRemove} onClose={onRemove} style={{width:'100%'}}>
                {displayText}
            </Tag>
            )}
            {displayText && isOver && canDrop && 'Replace'}
            {!displayText && isOver && canDrop && 'Place'}
            {!displayText && !isOver && <span style={{textTransform:'capitalize'}}>No {itemType}</span>}
        </div>
        
    )
};