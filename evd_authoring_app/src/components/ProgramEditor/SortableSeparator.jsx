import React from "react";
import { useDrop } from 'react-dnd';
import { PrimitiveBlock } from "./PrimitiveBlock";
import { UUIDBlock } from "./UUIDBlock";

export const SortableSeparator = ({ ancestors, context, onDrop, dropDisabled, height }) => {

    const [{ isOver, dragItem }, drop] = useDrop({
        accept: ancestors[0].accepts,
        drop: (item, _) => onDrop(item),
        canDrop: (item, _) => {
            if (!ancestors[0].accepts.some(type => type === item.type)) {
                return false
            } else if (!ancestors.some(ancestor => ancestor.uuid === item.parentData.uuid) && item.parentData.type !== 'drawer') {
                return false
            }
            return true;
        },
        collect: monitor => ({
            isOver: monitor.isOver(),
            dragItem: monitor.getItem()
        })
    })



    let contents = null;
    if (isOver && dragItem && !dropDisabled) {
        if (dragItem.type.includes('uuid')) {
            contents =
                <div style={{ opacity: 0.5 }} ref={dropDisabled ? null : drop} >
                    <UUIDBlock
                        ancestors={ancestors}
                        idx={0}
                        data={dragItem}
                        context={context}
                        dragDisabled
                        dropDisabled
                        dragBehavior='move' />
                </div>

        } else if (dragItem.type.includes('primitive')) {
            contents =
                <div style={{ opacity: 0.5 }} ref={dropDisabled ? null : drop} >
                    <PrimitiveBlock
                        ancestors={ancestors}
                        idx={0}
                        staticData={dragItem}
                        context={context}
                        dragDisabled
                        dropDisabled
                        dragBehavior='move' />
                </div>
        }
    }

    return (
        <div style={{ height: !contents ? 8 : null, width: '100%', paddingTop: contents ? 8 : 0, paddingBottom: contents ? 8 : 0 }}>
            {contents ? contents : (
                <div ref={dropDisabled ? null : drop} style={{ position: 'relative', zIndex: 80, top: -1*height/2, height }}></div>
            )}
        </div>

    )
};