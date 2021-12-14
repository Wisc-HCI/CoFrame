import React, {useCallback} from 'react';
import { PrimitiveBlock } from './PrimitiveBlock';
import { HierarchicalBlock } from './HierarchicalBlock';
import { SkillCallBlock } from './SkillCallBlock';
import { TrajectoryBlock } from './TrajectoryBlock';
import { UUIDBlock } from './UUIDBlock';
import { CanvasBlock } from './CanvasBlock';
import useStore from '../../../stores/Store';

const actionBlocks = ['delay','breakpoint','gripper','machine-initialize',
'process-start','process-stop','process-wait','move-trajectory','move-unplanned']

export const DisplayBlock = ({ancestors, uuid, staticData, context, dragDisabled, style}) => {

    const data = useStore(useCallback(state=>{
        if (staticData) {
            return staticData
        } else if (state.data[uuid]) {
            return state.data[uuid]
        } else if (context[uuid]) {
            return context[uuid]
        }
    },[uuid,staticData,context]))
    
    return (
        <React.Fragment>
            {data.type.includes('uuid-') ? (
                <UUIDBlock data={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled} style={style}/>
            ) : data.type === 'skill-call' ? (
                <SkillCallBlock data={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled} dropDisabled={true} style={style}/>
            ) : data.type === 'skill' ? (
                <CanvasBlock staticData={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled} dropDisabled={true} style={style}/>
            ) : data.type === 'hierarchical' ? (
                <HierarchicalBlock data={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled} dropDisabled={true} style={style}/>
            ) : data.type === 'trajectory' ? (
                <TrajectoryBlock data={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled} dropDisabled={true} style={style}/>
            ) : actionBlocks.includes(data.type) ? (
                <PrimitiveBlock data={data} ancestors={ancestors} context={context} dragDisabled={dragDisabled} style={style}/>
            ) : null }
        </React.Fragment>
    )
    
        
};