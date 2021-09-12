import React, {useCallback} from 'react';
import { GenericDraggable } from './Wrappers';
import { acceptLookup } from './acceptLookup';
import useStore from "../../stores/Store";
import { typeToKey, objectMap } from '../../stores/helpers';

export const UUIDDrawer = ({itemType}) => {
    // itemType is the "type" of item from the EvdStore that the uuid corresponds to.
    // e.g. 'thingType', 'machine', 'waypoint', etc.
    const data = useStore(useCallback(state=>
        Object.keys(state.data[typeToKey(itemType)])
        .map(uuid=>(state.data[typeToKey(itemType)][uuid]))
        .filter(data=>data.name.includes(state.searchTerm)),[itemType]));

    const nameLookup = useStore(state=>({
        ...objectMap(state.data.placeholders,placeholder=>({name:placeholder.pending_node.name,real:true})),
        ...objectMap(state.data.locations,location=>({name:location.name,real:true})),
        ...objectMap(state.data.waypoints,waypoint=>({name:waypoint.name,real:true})),
        ...objectMap(state.data.machines,machine=>({name:machine.name,real:true})),
        ...objectMap(state.data.trajectories,trajectory=>({name:trajectory.name,real:true})),
    }))

    const ancestors = [
        {uuid:'drawer',...acceptLookup.drawer.default}
    ];

    return (
        <React.Fragment>
            {data.map((datum)=>(
                <div key={datum.uuid} style={{paddingTop:5}} >
                    <GenericDraggable ancestors={ancestors} itemType='uuid' data={{...datum,itemType,type:`uuid-${itemType}`}} context={nameLookup}/>
                </div>
            ))}
        </React.Fragment>
    );
};
