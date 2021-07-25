import React from 'react';
import {useDrop} from 'react-dnd';
import { acceptLookup } from './acceptLookup';
import {ProgramBlock} from './ProgramBlock';
import {ItemDraggable} from './Wrappers';
import {Grid} from './Grid';
import useEvdStore from '../../stores/EvdStore';
import useGuiStore from '../../stores/GuiStore';

export const Canvas = (_) => {

    const clearFocusItem = useGuiStore(state=>state.clearFocusItem);
    const acceptTypes = acceptLookup.grid.primitiveIds.accepts;

    const ancestors = [
        {uuid:'grid',...acceptLookup.grid.primitiveIds}
    ];

    const [moveItem,skills] = useEvdStore(state=>[state.moveItem,state.data.skills]);

    // Do your draggable stuff here
    const [{},drop] = useDrop({
        accept: acceptTypes,
        drop: (item, monitor) => {
            const delta = monitor.getDifferenceFromInitialOffset();
            if (item.type === 'node.primitive.hierarchical.program.') {
                moveItem('program',item.uuid,delta.x,delta.y)
            } else if (item.type === 'node.primitive.hierarchical.skill.') {
                console.log('drop skill')
                moveItem('skill',item.uuid,delta.x,delta.y)
            } else {
                console.log(item)
            }
        },
        canDrop: (item, _) => (acceptTypes.indexOf(item.type)>=0)
    })

    return (
        <Grid ref={drop} onClick={clearFocusItem} >
            <ProgramBlock ancestors={ancestors}/>
            {Object.keys(skills).map(uuid=>(
                <ItemDraggable key={uuid} id={uuid} itemType='skill' data={skills[uuid]} ancestors={ancestors} />
            ))}
        </Grid>
    )
}
