import React from "react";
import { useDrop } from 'react-dnd';

const validDrop=(item,ancestors,count) => count === 0 && ancestors[0].accepts.indexOf(item.type)>=0 && (ancestors.map(ancestor=>ancestor.uuid).indexOf(item.parentData.uuid)>=0 || item.parentData.type === 'drawer');

export const NodeZone = ({ancestors,children,onMove,emptyMessage,enabled}) => {

    const drop = useDrop({
        accept: ancestors[0].accepts,
        drop: (item, _) => onMove(item),
        hover: (item, _) => {
          if (enabled && item.editable && validDrop(item,ancestors,children ? children.length: 0)) {onMove(item)}
        },
        canDrop: (item, _) => enabled && validDrop(item,ancestors,children ? children.length: 0)
      })[1]

    const containerStyle = {
        backgroundColor: 'rgba(0,0,0,0.5)',
        borderRadius: 5,
        minWidth: 20,
        minHeight:27,
        padding:3,
        textAlign:'center',
        fontSize:14
    }

    return (
        <div ref={drop} style={containerStyle}>
            {children && children.length === 0 ? emptyMessage : children}
        </div>
        
    )
};