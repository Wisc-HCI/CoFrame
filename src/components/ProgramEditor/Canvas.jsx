import React from 'react';
import { useDrop } from 'react-dnd';
import { acceptLookup } from './acceptLookup';
import useStore from '../../stores/Store';
import { CanvasBlock } from './Blocks/CanvasBlock';
import shallow from 'zustand/shallow';
import ReactFlow, {
    MiniMap,
    Controls,
    Background,
} from 'react-flow-renderer';
import lodash from 'lodash';

export const Canvas = ({bounding}) => {

    const ancestors = [
        { uuid: 'grid', ...acceptLookup.grid.children }
    ];

    const [flowInstance, setFlowInstance] = useStore(state=>[state.flowInstance,state.setFlowInstance],shallow)

    console.log(flowInstance)

    const [blocks, setEditorTransform, createAndPlaceItem, deleteBlock] = useStore(state => [
        lodash.filter(state.data, v => v.type === 'program' || v.type === 'skill'),
        state.setEditorTransform,
        state.createAndPlaceItem,
        state.deleteBlock
    ], shallow);
    const nameLookup = useStore(state => lodash.filter(state.data, v => ['thing', 'location', 'waypoint', 'machine', 'trajectory'].includes(v.type)), shallow)

    const parentData = { type: 'grid', uuid: 'grid' };

    const elements = blocks.map(b => ({
        id: b.uuid,
        type: b.type,
        data: { ancestors, nameLookup, parentData, uuid: b.uuid },
        position: b.transform,
        dragHandle: `.${b.uuid}`
    }));

    const CanvasBlockNode = ({ data }) => (
        <CanvasBlock
            onCanvas
            uuid={data.uuid}
            onDelete={(dropData) => deleteBlock(dropData, null)}
            parentData={data.parentData}
            dragBehavior='move'
            dragDisabled={false}
            dropDisabled={false}
            ancestors={data.ancestors}
            context={data.nameLookup}
        />
    )

    const [_, drop] = useDrop({
        accept: 'skill',
        drop: (item, monitor) => {

            const clientOffset = monitor.getClientOffset();
            const position = flowInstance.project({
                x: clientOffset.x - bounding.left + 170,
                y: clientOffset.y - bounding.top,
            });
            console.log({item,bounding,position})
            
            createAndPlaceItem(item.data,position.x,position.y)
        }
    })

    return (
        <ReactFlow
            ref={drop}
            maxZoom={1.5}
            nodesConnectable={false}
            elementsSelectable={false}
            nodeTypes={{ program: CanvasBlockNode, skill: CanvasBlockNode }}
            onMove={(transform) => { setEditorTransform(transform) }}
            elements={elements}
            onElementsRemove={elements => elements.forEach(element => deleteBlock('grid', element.id))}
            onConnect={(_) => { }}
            onLoad={(reactFlowInstance) => { 
                setFlowInstance(reactFlowInstance); 
                reactFlowInstance.fitView(); 
                reactFlowInstance.zoomTo(1)}}
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
                nodeBorderRadius={3}
            />
            <Controls />
            <Background variant='lines' color="#555" gap={40} />
        </ReactFlow>
    )
}
