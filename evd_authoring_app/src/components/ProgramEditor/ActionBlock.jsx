import React from 'react';
import useStore from '../../stores/Store';
import { PrimitiveBlock } from './PrimitiveBlock';
import { HierarchicalBlock } from './HierarchicalBlock';
import { SkillCallBlock } from './SkillCallBlock';

const COMPONENT_LOOKUP = {
    'node.primitive.delay.':PrimitiveBlock,
    'node.primitive.breakpoint':PrimitiveBlock,
    'node.primitive.gripper.':PrimitiveBlock,
    'node.primitive.machine-primitive.machine-initialize.':PrimitiveBlock,
    'node.primitive.machine-primitive.machine-start.':PrimitiveBlock,
    'node.primitive.machine-primitive.machine-stop.':PrimitiveBlock,
    'node.primitive.machine-primitive.machine-wait.':PrimitiveBlock,
    'node.primitive.move-trajectory.':PrimitiveBlock,
    'node.primitive.move-unplanned.':PrimitiveBlock,
    'node.primitive.skill-call.':SkillCallBlock,
    'node.primitive.hierarchical.':HierarchicalBlock,
}

export const ActionBlock = (props) => {

    const type = useStore(state=>{
        return props.staticData ? props.staticData.type : state.data.primitives[props.uuid].type
    })

    const Component = COMPONENT_LOOKUP[type];

    return <Component {...props} />
};