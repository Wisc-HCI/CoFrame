import { config, useSpring } from "@react-spring/core";
import { animated } from "@react-spring/web";
import React from "react";
import { useDrop } from 'react-dnd';
import useMeasure from "react-use-measure";
import { ActionBlock } from "./ActionBlock";
import { UUIDBlock } from "./UUIDBlock";

const validDrop = (dragItem, ancestors) => {
    if (!ancestors[0].accepts.some(type => type === dragItem.type)) {
        return false
    } else if (!ancestors.some(ancestor => ancestor.uuid === dragItem.parentData.uuid) && dragItem.parentData.type !== 'drawer') {
        return false
    }
    return true;
}

export const SortableSeparator = ({ ancestors, context, onDrop, dropDisabled, height, spacing, end }) => {

    const dropConfig = {
        accept: ancestors[0].accepts,
        drop: (item, _) => onDrop(item),
        canDrop: (item, _) => {
            if (dropDisabled) {
                return false
            } else {
                return validDrop(item, ancestors);
            }
        },
        collect: monitor => ({
            isOver: monitor.isOver(),
            dragItem: monitor.getItem()
        })
    }



    const [{ isOver, dragItem }, dropRef] = useDrop(dropConfig);
    // const [{ isOverDefault, dragItemDefault }, dropDefaultRef] = useDrop(dropConfig);
    // const [{ isOverPreview, dragItemPreview }, dropPreviewRef] = useDrop(dropConfig);

    // const isOver = isOverDefault || isOverPreview;
    // console.log(isOverDefault)
    // // console.log(isOver)
    // const dragItem = dragItemDefault ? dragItemDefault : dragItemPreview;
   
    const showPreview = isOver && dragItem && !dropDisabled && validDrop(dragItem, ancestors);
    // if (showPreview) { console.log('preview') }

    const [previewRef, shape] = useMeasure();
    const usedHeight = shape.height-height+spacing+5;
    const offset = end ? height/-2+spacing + 5 : height/-2+spacing;
    
    const previewStyle = useSpring({ height: usedHeight, config: config.stiff });

    return (
        <div 
            ref={dropRef} 
            style={{
                //...previewStyle,
                width: '100%', 
                position:'relative',
                // paddingTop: 0, 
                padding:0,
                height: showPreview ? null : spacing,
                // paddingBottom: showPreview ? 8 : 0, 
                // backgroundColor: 'blue' 
            }}>
            <div
                style={{
                    position: 'relative',
                    zIndex: 80,
                    top: -1 * height / 2,
                    opacity: 0.6,
                    // backgroundColor: 'red',
                    height: height/2
                }}>
            </div>
            <animated.div style={previewStyle}>
                <div ref={previewRef}>
                    {showPreview && dragItem.type.includes('uuid') && (
                        <div style={{ opacity: 0.5, top: offset, position:'relative'}}>
                            <UUIDBlock
                                ancestors={ancestors}
                                idx={0}
                                data={dragItem}
                                context={context}
                                dragDisabled
                                dropDisabled
                                dragBehavior='move' />
                        </div>
                    )}
                    {showPreview && dragItem.type.includes('primitive') && (
                        <div style={{ opacity: 0.5, top: offset, position:'relative'}}>
                            <ActionBlock
                                ancestors={ancestors}
                                idx={0}
                                locked
                                staticData={dragItem}
                                context={context}
                                dragDisabled
                                dragBehavior='move' />
                        </div>
                    )}
                </div>
            </animated.div>
            <div
                style={{
                    position: 'relative',
                    zIndex: 80,
                    bottom: height/2,
                    opacity: 0.6,
                    // backgroundColor: 'yellow',
                    height: height/2
                }}>
            </div>
        </div>

    )
};