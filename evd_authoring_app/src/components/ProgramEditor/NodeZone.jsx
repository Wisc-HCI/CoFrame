import React from "react";
import { useDrop } from 'react-dnd';
import { PrimitiveBlock } from "./PrimitiveBlock";
import { UUIDBlock } from "./UUIDBlock";

export const NodeZone = ({ancestors,children,context,onDrop,emptyMessage,dropDisabled,style}) => {

    const empty = !children || children.length === 0;

    const [{isOver,dragItem}, drop] = useDrop({
        accept: ancestors[0].accepts,
        drop: (item, _) => onDrop(item),
        canDrop: (item, _) => {
            if (!item) {
                return false
            } else if (!empty) {
                return false
            } else if (!ancestors[0].accepts.some(type=>type===item.type)) {
                return false
            } else if (!ancestors.some(ancestor=>ancestor.uuid===item.parentData.uuid) && item.parentData.type !== 'drawer') {
                return false
            }
            return true;
        },
        collect: monitor => ({
            isOver: monitor.isOver(),
            dragItem: monitor.getItem()
        })
      })

    

    let contents = empty ? (
        <div style={{padding:10}}>
            {emptyMessage}
        </div>
    ) : children;
    if (isOver && dragItem && !dropDisabled) {
        if (dragItem.type.includes('uuid')) {
            contents = 
            <UUIDBlock 
                ancestors={ancestors} 
                idx={0}
                data={dragItem} 
                context={context} 
                dragDisabled 
                dropDisabled
                dragBehavior='move' />
        } else if (dragItem.type.includes('primitive')) {
            contents = 
            <PrimitiveBlock
                ancestors={ancestors} 
                idx={0}
                data={dragItem} 
                context={context} 
                dragDisabled 
                dropDisabled
                dragBehavior='move' />
        }
    }

    const containerStyle = {
        backgroundColor: 'rgba(0,0,0,0.5)',
        borderRadius: 5,
        minWidth: 40,
        minHeight:58,
        paddingLeft:4,
        paddingRight:4,
        paddingTop:0,
        paddingBottom:0,
        textAlign:'center',
        fontSize:14
    }

    return (
        <div ref={dropDisabled||!empty?null:drop} style={{...containerStyle,...style,}}>
            {contents}
        </div>
        
    )
};