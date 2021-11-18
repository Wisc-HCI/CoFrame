import React, {useCallback} from 'react';
import { PrimitiveBlock } from './PrimitiveBlock';
import { HierarchicalBlock } from './HierarchicalBlock';
import { SkillCallBlock } from './SkillCallBlock';
import { TrajectoryBlock } from './TrajectoryBlock';
import { UUIDBlock } from './UUIDBlock';
import { CanvasBlock } from './CanvasBlock';
import { useDraggable } from '@dnd-kit/core';
import { CSS } from '@dnd-kit/utilities';
import useStore from '../../../stores/Store';

const actionBlocks = ['delay','breakpoint','gripper','machine-initialize',
'process-start','process-stop','process-wait','move-trajectory','move-unplanned']

export const Block = ({ancestors, uuid, staticData, context, dragDisabled, dragCopy}) => {

    const data = useStore(useCallback(state=>{
        if (staticData) {
            return staticData
        } else if (state.data[uuid]) {
            return state.data[uuid]
        } else if (context[uuid]) {
            return context[uuid]
        }
    },[uuid,staticData,context]))

    const dragData = useStore(state=>state.dragData);

    const {
        attributes,
        listeners,
        setNodeRef,
        transform,
        transition,
    } = useDraggable({
        id: data.uuid, 
        data: {data,context,ancestors},
        disabled: dragDisabled
    });



    const style = {
        transform: CSS.Transform.toString(transform),
        transition,
    };

    return (
        <div hidden={false && dragData?.data?.uuid === data.uuid && !dragCopy} ref={setNodeRef} style={style} role='button' {...attributes} >
            {data.type.includes('uuid-') ? (
                <UUIDBlock data={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled} listeners={listeners}/>
            ) : data.type === 'skill-call' ? (
                <SkillCallBlock data={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled} listeners={listeners}/>
            ) : data.type === 'skill' ? (
                <CanvasBlock staticData={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled} listeners={listeners}/>
            ) : data.type === 'hierarchical' ? (
                <HierarchicalBlock data={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled} listeners={listeners}/>
            ) : data.type === 'trajectory' ? (
                <TrajectoryBlock data={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled} listeners={listeners}/>
            ) : actionBlocks.includes(data.type) ? (
                <PrimitiveBlock data={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled} listeners={listeners}/>
            ) : null }
        </div>
    )
    
        
};