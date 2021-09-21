import React, {useCallback} from 'react';
// import { StaticSortable } from './Wrappers';
import { acceptLookup } from './acceptLookup';
import useStore from "../../stores/Store";
import shallow from 'zustand/shallow';
import { typeToKey, objectMap } from '../../stores/helpers';
import { UUIDBlock } from './UUIDBlock';

export const UUIDDrawer = ({itemType}) => {
    // itemType is the "type" of item from the EvdStore that the uuid corresponds to.
    // e.g. 'thingType', 'machine', 'waypoint', etc.
    const data = useStore(useCallback(state=>
        Object.values(state.data[typeToKey(itemType)])
        .filter(data=>data.name.includes(state.searchTerm)),[itemType]),shallow);

    const nameLookup = useStore(state=>({
        ...objectMap(state.data.placeholders,placeholder=>({name:placeholder.pending_node.name,real:true})),
        ...objectMap(state.data.locations,location=>({name:location.name,real:true})),
        ...objectMap(state.data.waypoints,waypoint=>({name:waypoint.name,real:true})),
        ...objectMap(state.data.machines,machine=>({name:machine.name,real:true})),
        ...objectMap(state.data.trajectories,trajectory=>({name:trajectory.name,real:true})),
    }),shallow)

    const ancestors = [
        {uuid:'drawer',...acceptLookup.drawer.default}
    ];

    return (
        <React.Fragment>
            {data.map((datum,i)=>(
                <div key={datum.uuid} style={{paddingTop:5}} >
                    <UUIDBlock 
                        ancestors={ancestors} 
                        idx={i} 
                        parentData={{type:'drawer',uuid:'drawer'}}
                        data={{...datum,itemType,type:`uuid-${itemType}`}} 
                        dragBehavior='copy'
                        dropDisabled
                        context={nameLookup}/>
                </div>
            ))}
        </React.Fragment>
    );
};
