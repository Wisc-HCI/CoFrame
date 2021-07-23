import React from 'react';
import { Tag } from 'antd';
import { useDrop } from 'react-dnd';

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
        backgroundColor: 'grey',
        boxShadow:'inset 0pt 0pt 2pt 1pt rgba(0,0,0,0.5)',
        borderRadius: 5,
        minWidth: 20,
        minHeight:25,
        padding:3,
        textAlign:'center'
    }

    const itemTypeColorLookup = {
        waypoint: '#AD1FDE',
        thing: '#E08024',
        location: '#8624E0',
        machine: '#B3A533'
    }

    return (
        <div ref={drop} style={containerStyle} >
            {displayText && (!isOver || !canDrop) && (
            <Tag color={itemTypeColorLookup[itemType]} closable={canRemove} onClose={onRemove} style={{width:'100%'}}>
                {displayText}
            </Tag>
            )}
            {displayText && isOver && canDrop && 'Replace'}
            {!displayText && isOver && canDrop && 'Place'}
            {!displayText && !isOver && <span style={{textTransform:'capitalize'}}>No {itemType}</span>}
        </div>
        
    )
};