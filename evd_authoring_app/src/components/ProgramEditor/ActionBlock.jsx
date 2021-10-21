import React from 'react';
import useStore from '../../stores/Store';
import { PrimitiveBlock } from './PrimitiveBlock';
import { HierarchicalBlock } from './HierarchicalBlock';
import { SkillCallBlock } from './SkillCallBlock';

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
}

export const ActionBlock = (props) => {

    const type = useStore(state=>{
        return props.staticData ? props.staticData.type : state.data[props.uuid]?.type
    })
    
    if (!type) {
        return null
    }
    const Component = COMPONENT_LOOKUP[type];

    return <Component {...props} />
};