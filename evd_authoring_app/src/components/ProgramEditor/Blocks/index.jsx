import React, {useCallback} from 'react';
import { PrimitiveBlock } from './PrimitiveBlock';
import { HierarchicalBlock } from './HierarchicalBlock';
import { SkillCallBlock } from './SkillCallBlock';
import { TrajectoryBlock } from './TrajectoryBlock';
import { UUIDBlock } from './UUIDBlock';
import { useSortable } from '@dnd-kit/sortable';
import { CSS } from '@dnd-kit/utilities';
import useStore from '../../../stores/Store';

const COMPONENT_LOOKUP = {
    'delay':PrimitiveBlock,
    'breakpoint':PrimitiveBlock,
    'gripper':PrimitiveBlock,
    'machine-initialize':PrimitiveBlock,
    'process-start':PrimitiveBlock,
    'process-stop':PrimitiveBlock,
    'process-wait':PrimitiveBlock,
    'move-trajectory':PrimitiveBlock,
    'move-unplanned':PrimitiveBlock,
    'skill-call':SkillCallBlock,
    'hierarchical':HierarchicalBlock,
    'trajectory':TrajectoryBlock,
    'uuid-machine':UUIDBlock,
    'uuid-trajectory':UUIDBlock,
    'uuid-location':UUIDBlock,
    'uuid-waypoint':UUIDBlock,
    'uuid-thing':UUIDBlock
}

export const Block = ({ancestors, uuid, staticData, context, dragDisabled}) => {

    const data = useStore(useCallback(state=>{
        if (staticData) {
            return staticData
        } else if (state.data[uuid]) {
            console.log('found in store')
            return state.data[uuid]
        } else if (context[uuid]) {
            return context[uuid]
        }
    },[uuid,staticData,context]))

    console.log({staticData,uuid,data})

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
    
    const Component = COMPONENT_LOOKUP[data.type];

    return (
        <div ref={setNodeRef} style={style} {...attributes} {...listeners}>
            <Component data={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled}/>
        </div>
    )
    
        
};