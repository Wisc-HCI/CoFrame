import React, { useRef, useState, useEffect } from 'react';
import { useDrop } from 'react-dnd';
import { acceptLookup } from './acceptLookup';
import { ProgramBlock } from './ProgramBlock';
// import {ItemDraggable} from './Wrappers';
import { Grid } from './Grid';
import useStore from '../../stores/Store';
import { objectMap } from '../../stores/helpers';
import { SkillBlock } from './SkillBlock';


export const Canvas = () => {

    const ref = useRef();
    const [clearFocusItem] = useStore(state => [state.clearFocusItem]);
    const acceptTypes = acceptLookup.grid.primitiveIds.accepts;

    const ancestors = [
        { uuid: 'grid', ...acceptLookup.grid.primitiveIds }
    ];

    const [moveItem, createAndPlaceItem, skills] = useStore(state => [state.moveItem, state.createAndPlaceItem, state.data.skills]);
    const nameLookup = useStore(state => ({
        ...objectMap(state.data.placeholders, placeholder => ({ name: placeholder.pending_node.name, real: true })),
        ...objectMap(state.data.locations, location => ({ name: location.name, real: true })),
        ...objectMap(state.data.waypoints, waypoint => ({ name: waypoint.name, real: true })),
        ...objectMap(state.data.machines, machine => ({ name: machine.name, real: true })),
        ...objectMap(state.data.trajectories, trajectory => ({ name: trajectory.name, real: true })),
    }))

    // Do your draggable stuff here
    // We only care about the second value returned from useDrop (hence the [1] at the end)
    const drop = useDrop({
        accept: acceptTypes,
        drop: (item, monitor) => {
            const delta = monitor.getDifferenceFromInitialOffset();
            if (item.type === 'node.primitive.hierarchical.program.') {
                moveItem('program', item.uuid, delta.x, delta.y)
            } else if (item.type === 'node.primitive.hierarchical.skill.' && skills[item.uuid]) {
                moveItem('skill', item.uuid, delta.x, delta.y)
            } else if (item.type === 'node.primitive.hierarchical.skill.') {
                const offset = monitor.getClientOffset();
                const rect = ref.current.getBoundingClientRect();
                createAndPlaceItem('skill', item, (offset.x - rect.x), (offset.y - rect.y))
            } else {
                console.log(item)
            }
        },
        canDrop: (item, _) => (acceptTypes.indexOf(item.type) >= 0)
    })[1]

    return (
        <div style={{width:'100%',height:'100%',zIndex:100,position:'relative',overflow:'scroll',textAlign:'left'}}>
                <Grid ref={drop(ref)} onClick={clearFocusItem} >
                    <ProgramBlock
                            ancestors={ancestors}
                            context={nameLookup}
                            dragBehavior='move'
                            parentData={{ type: 'grid', uuid: 'grid' }}
                        />
                        {Object.keys(skills).map(uuid => (
                            <SkillBlock
                                key={uuid}
                                uuid={uuid}
                                parentData={{ type: 'grid', uuid: 'grid' }}
                                dragBehavior='move'
                                ancestors={ancestors}
                                context={nameLookup} />
                        ))}
                </Grid>
            
            
        </div>
        

    )
}
