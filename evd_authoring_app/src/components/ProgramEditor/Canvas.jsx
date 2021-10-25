import React, { useRef, useState } from 'react';
// import { useDrop } from 'react-dnd';
import { acceptLookup } from './acceptLookup';
// import { ProgramBlock } from './ProgramBlock';
// import {ItemDraggable} from './Wrappers';
// import { Grid } from './Grid';
import useStore from '../../stores/Store';
// import { objectMap } from '../../stores/helpers';
import { CanvasBlock } from './CanvasBlock';
import shallow from 'zustand/shallow';
import ReactFlow, {
    removeElements,
    addEdge,
    MiniMap,
    Controls,
    Background,
} from 'react-flow-renderer';
import lodash from 'lodash';

export const Canvas = () => {

    // const ref = useRef();
    // const clearFocusItem = useStore(state => state.clearFocusItem);
    // const acceptTypes = acceptLookup.grid.children.accepts;

    const ancestors = [
        { uuid: 'grid', ...acceptLookup.grid.children }
    ];

    const [blocks, setEditorTransform, moveItem, createAndPlaceItem, deleteBlock] = useStore(state => [
        lodash.filter(state.data,v=>v.type === 'program' || v.type === 'skill'), 
        state.setEditorTransform,
        state.moveItem, 
        state.createAndPlaceItem, 
        state.deleteBlock
    ], shallow);
    const nameLookup = useStore(state => lodash.filter(state.data, v=>['thing','location','waypoint','machine','trajectory'].includes(v.type)), shallow)

    // Do your draggable stuff here
    // We only care about the second value returned from useDrop (hence the [1] at the end)
    // const drop = useDrop({
    //     accept: acceptTypes,
    //     drop: (item, monitor) => {
    //         const delta = monitor.getDifferenceFromInitialOffset();
    //         if (item.type === 'program') {
    //             moveItem(item.uuid, delta.x, delta.y)
    //         } else if (item.type === 'skill' && skills[item.uuid]) {
    //             moveItem('skill', item.uuid, delta.x, delta.y)
    //         } else if (item.type === 'node.primitive.hierarchical.skill.') {
    //             const offset = monitor.getClientOffset();
    //             const rect = ref.current.getBoundingClientRect();
    //             createAndPlaceItem('skill', item, (offset.x - rect.x), (offset.y - rect.y))
    //         } else {
    //             console.log(item)
    //         }
    //     },
    //     canDrop: (item, _) => (acceptTypes.indexOf(item.type) >= 0)
    // })[1]
    
    const parentData = { type: 'grid', uuid: 'grid' };

    const CanvasBlockNode = ({data}) => (
        <CanvasBlock
            uuid={data.uuid}
            onDelete={(dropData)=>deleteBlock(dropData,null)}
            parentData={data.parentData}
            dragBehavior='move'
            ancestors={data.ancestors}
            context={data.nameLookup} 
        />
    )

    const elements = blocks.map(b=>({
        id: b.uuid, 
        type: b.type,
        data: {ancestors,nameLookup,parentData,uuid:b.uuid},
        position: b.transform
    }));

    return (
        <ReactFlow
            maxZoom={1.5}
            nodeTypes={{program: CanvasBlockNode, skill: CanvasBlockNode}}
            onMove={(transform)=>{setEditorTransform(transform)}}
            elements={elements}
            onElementsRemove={elements=>elements.forEach(element=>deleteBlock('grid', element.id))}
            onConnect={(_) => {}}
            onLoad={(reactFlowInstance) => {reactFlowInstance.fitView()}}
            snapToGrid={true}
            snapGrid={[15, 15]}
        >
            <MiniMap
                maskColor="#1a192b44"
                nodeStrokeColor={(n) => {
                    if (n.style?.background) return n.style.background;
                    if (n.type === 'program') return '#3f3f3f';
                    if (n.type === 'skill') return '#62869e';

                    return '#eee';
                }}
                nodeColor={(n) => {
                    if (n.type === 'program') return '#3f3f3f';
                    if (n.type === 'skill') return '#62869e';
                    if (n.style?.background) return n.style.background;

                    return '#fff';
                }}
                nodeBorderRadius={2}
            />
            <Controls />
            <Background variant='lines' color="#555" gap={40} />
        </ReactFlow>
    )
}
