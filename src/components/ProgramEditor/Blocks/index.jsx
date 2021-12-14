import React, {useCallback} from 'react';
import { PrimitiveBlock } from './PrimitiveBlock';
import { HierarchicalBlock } from './HierarchicalBlock';
import { SkillCallBlock } from './SkillCallBlock';
import { TrajectoryBlock } from './TrajectoryBlock';
import { UUIDBlock } from './UUIDBlock';
import { CanvasBlock } from './CanvasBlock';
// import { useDraggable } from '@dnd-kit/core';
// import { CSS } from '@dnd-kit/utilities';
import { useDrag } from 'react-dnd';
import useStore from '../../../stores/Store';

const actionBlocks = ['delay','breakpoint','gripper','machine-initialize',
'process-start','process-stop','process-wait','move-trajectory','move-unplanned']

export const Block = ({ancestors, uuid, staticData, context, dragDisabled, dropDisabled, dragCopy, field, idx, fromNodeList}) => {

    const data = useStore(useCallback(state=>{
        if (staticData) {
            return staticData
        } else if (state.data[uuid]) {
            return state.data[uuid]
        } else if (context[uuid]) {
            return context[uuid]
        }
    },[uuid,staticData,context]))

    const setActiveDrawer = useStore(store=>store.setActiveDrawer)

    const [dragProps, drag, preview] = useDrag({
        type: data.type,
        item: () => {
            setTimeout(()=>setActiveDrawer(null),50)
            return {data,context,ancestors,fieldInfo:{field,idx,uuid:ancestors[0].uuid}}
        },
        canDrag: !dragDisabled,
        options: { dragEffect: dragCopy ? 'copy' : 'move' },
        collect: (monitor) => {
          return {
            isDragging: monitor.isDragging()
          }
        },
    })

    if (fromNodeList) {console.log({dragDisabled,dropDisabled,...dragProps,data})}



    return (
        <div hidden={dragProps.isDragging && !dragCopy} ref={preview}>
            {data.type.includes('uuid-') ? (
                <UUIDBlock 
                    data={data}
                    dragHandle={drag}
                    ancestors={ancestors}
                    context={context} 
                    dragDisabled={dragDisabled}
                    dropDisabled={dropDisabled || dragProps.isDragging}/>
            ) : data.type === 'skill-call' ? (
                <SkillCallBlock 
                    data={data} 
                    dragHandle={drag}
                    ancestors={ancestors} 
                    context={context} 
                    dragDisabled={dragDisabled}
                    dropDisabled={dropDisabled || dragProps.isDragging}/>
            ) : data.type === 'skill' ? (
                <CanvasBlock 
                    staticData={data} 
                    dragHandle={drag}
                    ancestors={ancestors} 
                    context={context} 
                    dragDisabled={dragDisabled}
                    dropDisabled={dropDisabled || dragProps.isDragging}/>
            ) : data.type === 'hierarchical' ? (
                <HierarchicalBlock 
                    data={data} 
                    dragHandle={drag}
                    ancestors={ancestors} 
                    context={context} 
                    dragDisabled={dragDisabled}
                    dropDisabled={dropDisabled || dragProps.isDragging}/>
            ) : data.type === 'trajectory' ? (
                <TrajectoryBlock 
                    data={data} 
                    dragHandle={drag}
                    ancestors={ancestors}
                    context={context} 
                    dragDisabled={dragDisabled}
                    dropDisabled={dropDisabled || dragProps.isDragging}/>
            ) : actionBlocks.includes(data.type) ? (
                <PrimitiveBlock 
                    data={data} 
                    dragHandle={drag}
                    ancestors={ancestors} 
                    context={context} 
                    dragDisabled={dragDisabled}
                    dropDisabled={dropDisabled || dragProps.isDragging}/>
            ) : null }
        </div>
    )
    
        
};