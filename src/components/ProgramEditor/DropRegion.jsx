import React from 'react';
import { useDrop } from 'react-dnd';
import { DisplayBlock } from "./Blocks/DisplayBlock";
import useStore from '../../stores/Store';
import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';


function validType(item, ancestors) {
    return ancestors[0].accepts.includes(item?.data?.type)
}

function validAncestor(item, ancestors) {
    return item?.ancestors[0].uuid === 'drawer' || ancestors.map(a => a.uuid).includes(item?.ancestors[0].uuid)
}

export const DropRegion = ({ parentId, field, idx, ancestors, height, text, dropDisabled }) => {

    const dropData = { uuid: parentId, field, idx };

    const transferBlock = useStore(store => store.transferBlock);

    const inDrawer = ancestors.map(a => a.uuid).includes('drawer')

    const [dropProps, drop] = useDrop({
        accept: ancestors[0].accepts,
        drop: (item, _) => transferBlock(item.data, item.fieldInfo, dropData),
        canDrop: (item, _) => {
            if (dropDisabled || inDrawer) {
                return false
            } else if (!item) {
                return false
            } else {
                return validType(item, ancestors) && validAncestor(item, ancestors)
            }
        },
        collect: monitor => ({
            isOver: monitor.isOver(),
            dragItem: monitor.getItem(),
            accepts: ancestors[0].accepts,
            validType: !inDrawer && validType(monitor.getItem(), ancestors),
            validAncestor: !inDrawer && validAncestor(monitor.getItem(), ancestors),
            validHover: !inDrawer && !dropDisabled && monitor.isOver() && validType(monitor.getItem(), ancestors) && validAncestor(monitor.getItem(), ancestors)
        })
    })

    // console.log(dropProps)

    const dropStyle = useSpring({
        opacity: dropProps.validHover ? 0.5 : 0,
        scaleY: dropProps.validHover ? 1 : 0,
        config: config.stiff
    });

    const showPreview = dropProps.validHover;
    const showHalo = !dropProps.validHover && dropProps.validAncestor && dropProps.validType;

    const styles = {
        borderRadius: 5,
        minHeight: height,
        textAlign: 'center',
        fontSize: 14,
        boxShadow: showHalo ? 'inset 0px 0px 2px 1px #ffffff' : null
    }

    return (
        <div style={{paddingTop: showPreview || showHalo ? 6 : 0, paddingBottom: showPreview || showHalo ? 6 : 0}}>
            <div className='nodrag' ref={drop} style={styles}>
                <animated.div style={{ ...dropStyle }} >
                    {dropProps.validHover && (
                        <DisplayBlock
                            ancestors={dropProps.dragItem.ancestors}
                            staticData={dropProps.dragItem.data}
                            context={dropProps.dragItem.context}
                            dragDisabled={true}
                            style={{ opacity: 0.6 }}
                        />

                    )}
                </animated.div>
                {!dropProps.validHover && (
                    <span>{text}</span>
                )}
            </div>
        </div>



    )
};

