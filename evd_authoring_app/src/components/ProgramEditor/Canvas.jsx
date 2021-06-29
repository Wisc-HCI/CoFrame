import React from 'react';
import {useDrop} from 'react-dnd';
import { acceptLookup } from './acceptLookup';
import {ProgramBlock} from './ProgramBlock';
import {SkillBlock} from './SkillBlock';
import {Grid} from './Grid';
import {DeleteZone} from './DeleteZone';
import useEvdStore from '../../stores/EvdStore';

export const Canvas = (_) => {

    const acceptTypes = acceptLookup.grid.primitiveIds.accepts;

    const ancestors = [
        {uuid:'grid',...acceptLookup.grid.primitiveIds}
    ];

    const [moveItem] = useEvdStore(state=>[state.moveItem]);

    // Do your draggable stuff here
    const [{}, drop] = useDrop({
        accept: acceptTypes,
        drop: (item, monitor) => {
            const delta = monitor.getDifferenceFromInitialOffset();
            if (item.type === 'node.primitive.hierarchical.program.') {
                moveItem('program',item.uuid,delta.x,delta.y)
            } else if (item.type === 'node.primitive.hierarchical.skill.') {
                moveItem('skill',item.uuid,delta.x,delta.y)
            }
        },
        canDrop: (item, _) => (acceptTypes.indexOf(item.type)>=0),
        collect: (monitor) => ({
            isOver: monitor.isOver(),
            canDrop: monitor.canDrop()
        })
    })

    return (
        <Grid ref={drop}>
            <ProgramBlock ancestors={ancestors}/>
            {/* Put the skills in here too */}
        </Grid>
    )
}
