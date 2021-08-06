import React, {useRef} from 'react';
import {useDrop} from 'react-dnd';
import { acceptLookup } from './acceptLookup';
import {ProgramBlock} from './ProgramBlock';
import {ItemDraggable} from './Wrappers';
import {Grid} from './Grid';
import useEvdStore from '../../stores/EvdStore';
import useGuiStore from '../../stores/GuiStore';

export const Canvas = () => {

    const ref = useRef();
    const clearFocusItem = useGuiStore(state=>state.clearFocusItem);
    const acceptTypes = acceptLookup.grid.primitiveIds.accepts;

    const ancestors = [
        {uuid:'grid',...acceptLookup.grid.primitiveIds}
    ];

    const [moveItem,createAndPlaceItem,skills] = useEvdStore(state=>[state.moveItem,state.createAndPlaceItem,state.data.skills]);

    // Do your draggable stuff here
    // We only care about the second value returned from useDrop (hence the [1] at the end)
    const drop = useDrop({
        accept: acceptTypes,
        drop: (item, monitor) => {
            const delta = monitor.getDifferenceFromInitialOffset();

            const offset = monitor.getClientOffset();
            const rect = ref.current.getBoundingClientRect();
            console.log(item.transform)
            const newX = offset.x-rect.x;
            const newY = offset.y-rect.y;
            console.log({x:newX,y:newY})
            console.log({x:newX-item.transform.x,y:newY-item.transform.y})


            if (item.type === 'node.primitive.hierarchical.program.') {
                moveItem('program',item.uuid,delta.x,delta.y)
            } else if (item.type === 'node.primitive.hierarchical.skill.' && item.parentData) {
                const offset = monitor.getClientOffset();
                const rect = ref.current.getBoundingClientRect();

                createAndPlaceItem('skill',item,offset.x-rect.x,offset.y-rect.y)
            } else if (item.type === 'node.primitive.hierarchical.skill.') {
                console.log('drop skill')
                moveItem('skill',item.uuid,delta.x,delta.y)
            } else {
                console.log(item)
            }
        },
        canDrop: (item, _) => (acceptTypes.indexOf(item.type)>=0)
    })[1]

    return (
        <Grid ref={drop(ref)} onClick={clearFocusItem} >
            <ProgramBlock ancestors={ancestors}/>
            {Object.keys(skills).map(uuid=>(
                <ItemDraggable key={uuid} id={uuid} itemType='skill' data={skills[uuid]} ancestors={ancestors} />
            ))}
        </Grid>
    )
}
