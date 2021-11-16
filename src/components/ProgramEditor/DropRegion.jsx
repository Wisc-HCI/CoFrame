import React from 'react';
import { useDroppable } from '@dnd-kit/core';
import { DisplayBlock } from "./Blocks/DisplayBlock";
import useStore from '../../stores/Store';

export const DropRegion = ({parentId, field, idx, ancestors, height, text, dropDisabled}) => {

    const dragData = useStore(state=>state.dragData);
    const dropData = useStore(state=>state.dropData);

    const validType = ancestors[0].accepts.includes(dragData?.data?.type);
    const ancestorChecked = ancestors.map(a=>a.uuid).includes(dragData?.ancestors[0].uuid || dragData?.ancestors[0].uuid === 'grid');
    const isDrop = dropData?.uuid === parentId && dropData?.field === field && dropData?.idx === idx;
    const validHover = isDrop && validType && !dropDisabled;

    const {setNodeRef} = useDroppable({
        id:JSON.stringify({parentId,field,idx}),
        data: {uuid: parentId, field, idx},
        disabled: (validType && ancestorChecked)  || dropDisabled
    });
    
    // console.log({dropData,dragData})
    // console.log(ancestors[0].accepts)
    // console.log(ancestors[0].accepts.includes(dragData?.data?.type))
    // console.log(validHover)

    const styles = {
        borderRadius: 5,
        minHeight: height,
        textAlign: 'center',
        fontSize: 14,
        boxShadow: validType && !dropDisabled ? 'inset 0px 0px 2px 1px #ffffff' : null
    }

    return (
        <div className='nodrag' ref={setNodeRef} style={styles} >
            {validHover ? (
                <DisplayBlock ancestors={dragData.ancestors} staticData={dragData.data} context={dragData.context} dragDisabled={true}/>
            ) : (
                <span>{text}</span>
            )}
        </div>
        
    )
};

