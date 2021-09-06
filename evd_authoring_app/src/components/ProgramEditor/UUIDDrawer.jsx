import React, {useCallback} from 'react';
import { GenericDraggable } from './Wrappers';
import { acceptLookup } from './acceptLookup';
import useStore from "../../stores/Store";
import { typeToKey } from '../../stores/helpers';

export const UUIDDrawer = ({itemType}) => {
    // itemType is the "type" of item from the EvdStore that the uuid corresponds to.
    // e.g. 'thingType', 'machine', 'waypoint', etc.
    const data = useStore(useCallback(state=>
        Object.keys(state.data[typeToKey(itemType)])
        .map(uuid=>(state.data[typeToKey(itemType)][uuid]))
        .filter(data=>data.name.includes(state.searchTerm)),[itemType]));

    const ancestors = [
        {uuid:'drawer',...acceptLookup.drawer.default}
    ];

    return (
        <React.Fragment>
            {data.map((datum)=>(
                <div key={datum.uuid} style={{paddingTop:5}} >
                    <GenericDraggable ancestors={ancestors} itemType='uuid' data={{...datum,itemType,type:`uuid-${itemType}`}}/>
                </div>
            ))}
        </React.Fragment>
    );
};
