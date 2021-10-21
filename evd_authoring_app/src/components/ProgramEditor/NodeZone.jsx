import React from "react";
import { useDrop } from 'react-dnd';
import { ActionBlock } from "./ActionBlock";
import { TrajectoryBlock } from "./TrajectoryBlock";
import { UUIDBlock } from "./UUIDBlock";

const validDrop = (item, ancestors) => {
    if (!ancestors[0].accepts.some(type => type === item.type)) {
        return false
    } else if (!ancestors.some(ancestor => ancestor.uuid === item.parentData.uuid) && item.parentData.type !== 'drawer') {
        return false
    }
    return true;
}

export const NodeZone = ({ ancestors, children, context, onDrop, emptyMessage, dropDisabled, style }) => {

    const empty = !children || children.length === 0;

    const [{ isOver, dragItem }, drop] = useDrop({
        accept: ancestors[0].accepts,
        drop: (item, _) => onDrop(item),
        canDrop: (item, _) => {
            if (!item) {
                return false
                // } else if (dropDisabled||!empty) {
                //     return false
            } else if (!empty) {
                return false
            } else {
                return validDrop(item, ancestors)
            }
        },
        collect: monitor => ({
            isOver: monitor.isOver(),
            dragItem: monitor.getItem(),
            // canDrop: monitor.canDrop()
        })
    })

    const containerStyle = {
        backgroundColor: 'rgba(0,0,0,0.5)',
        borderRadius: 5,
        minWidth: 110,
        minHeight: 54,
        padding: 5,
        textAlign: 'center',
        fontSize: 14
    }

    const showPreview = isOver && dragItem && !dropDisabled && validDrop(dragItem, ancestors);
    const showChildren = !empty && !showPreview;

    return (
        <div className="nodrag" ref={dropDisabled || !empty ? null : drop} style={{ ...containerStyle, ...style, }}>
            {!showChildren && !showPreview && (
                <div style={{ padding: 10 }}>
                    {emptyMessage}
                </div>
            )}
            {!showPreview && showChildren && children}
            {!showChildren && showPreview && dragItem.type.includes('uuid') && (
                <UUIDBlock
                    ancestors={ancestors}
                    idx={0}
                    data={dragItem}
                    context={context}
                    dragDisabled
                    dropDisabled
                    dragBehavior='move' />
            )}
            {!showChildren && showPreview && dragItem.type.includes('primitive') && (
                <ActionBlock
                    ancestors={ancestors}
                    idx={0}
                    staticData={dragItem}
                    context={context}
                    dragDisabled
                    dropDisabled
                    locked
                    dragBehavior='move' />
            )}
            {!showChildren && showPreview && !dragItem.type.includes('uuid') && !dragItem.type.includes('primitive') && dragItem.type.includes('trajectory') && (
                <TrajectoryBlock
                    staticData={dragItem}
                    ancestors={ancestors}
                    dragDisabled
                    context={context}
                />
            )}

        </div>

    )
};