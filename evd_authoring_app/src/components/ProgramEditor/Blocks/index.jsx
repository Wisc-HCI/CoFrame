import React, {useCallback} from 'react';
import { PrimitiveBlock } from './PrimitiveBlock';
import { HierarchicalBlock } from './HierarchicalBlock';
import { SkillCallBlock } from './SkillCallBlock';
import { TrajectoryBlock } from './TrajectoryBlock';
import { UUIDBlock } from './UUIDBlock';
import { CanvasBlock } from './CanvasBlock';
import { useSortable } from '@dnd-kit/sortable';
import { CSS } from '@dnd-kit/utilities';
import useStore from '../../../stores/Store';

const actionBlocks = ['delay','breakpoint','gripper','machine-initialize',
'process-start','process-stop','process-wait','move-trajectory','move-unplanned']

export const Block = ({ancestors, uuid, staticData, context, dragDisabled}) => {

    const data = useStore(useCallback(state=>{
        if (staticData) {
            return staticData
        } else if (state.data[uuid]) {
            return state.data[uuid]
        } else if (context[uuid]) {
            return context[uuid]
        }
    },[uuid,staticData,context]))
    
    const {
        attributes,
        listeners,
        setNodeRef,
        transform,
        transition,
    } = useSortable({id: data.uuid, disabled: dragDisabled});

    const style = {
        transform: CSS.Transform.toString(transform),
        transition,
    };

    return (
        <div ref={setNodeRef} style={{...style,marginBottom:4}} {...attributes} {...listeners}>
            {data.type.includes('uuid-') ? (
                <UUIDBlock data={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled}/>
            ) : data.type === 'skill-call' ? (
                <SkillCallBlock data={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled}/>
            ) : data.type === 'skill' ? (
                <CanvasBlock staticData={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled}/>
            ) : data.type === 'hierarchical' ? (
                <HierarchicalBlock data={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled}/>
            ) : data.type === 'trajectory' ? (
                <TrajectoryBlock data={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled}/>
            ) : actionBlocks.includes(data.type) ? (
                <PrimitiveBlock data={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled}/>
            ) : null }
        </div>
    )
    
        
};