import React from 'react';
import {useDroppable} from '@dnd-kit/core';
import { acceptLookup } from './acceptLookup';
// import { ItemDraggable } from './Wrappers';
import {ProgramDraggable} from './Wrappers/ProgramDraggable';
import {Grid} from './Grid'

export const Canvas = (_) => {

    const ancestors = [
        {uuid:'grid',...acceptLookup.grid.primitiveIds}
    ];
    // Do your draggable stuff here
    const {setNodeRef} = useDroppable(
        {id: 'grid', data:{uuid:'grid',idx:0,ancestors,itemType:'grid',action:null}}
    );
    return (
        <Grid ref={setNodeRef}>
            <ProgramDraggable/>
            {/* Put the skills in here too */}
        </Grid>
    )
}
