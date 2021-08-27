import React, {useCallback} from 'react';
// import {
//     SortableContext,
//     verticalListSortingStrategy,
// } from '@dnd-kit/sortable';
import { GenericDraggable } from './Wrappers';
import { acceptLookup } from './acceptLookup';
import useStore from "../../stores/Store";
import { typeToKey } from '../../stores/helpers';

export const UUIDDrawer = ({itemType}) => {
    // itemType is the "type" of item from the EvdStore that the uuid corresponds to.
    // e.g. 'thingType', 'machine', 'waypoint', etc.

    const uuids = useStore(useCallback(state=>Object.keys(state.data[typeToKey(itemType)]),[itemType]));
    
    const ancestors = [
        {uuid:'drawer',...acceptLookup.drawer.default}
    ];

    return (
        <React.Fragment>
            {uuids.map((uuid)=>(
                <div key={uuid} style={{paddingTop:5}} >
                    <GenericDraggable ancestors={ancestors} itemType='uuid' data={{itemType,uuid,type:`uuid-${itemType}`}}/>
                </div>
            ))}
        </React.Fragment>
    );
};
