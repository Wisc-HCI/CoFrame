import React, {useCallback} from 'react';
// import { StaticSortable } from './Wrappers';
import { acceptLookup } from '../acceptLookup';
import useStore from "../../../stores/Store";
import shallow from 'zustand/shallow';
// import { typeToKey, objectMap } from '../../stores/helpers';
import { generateUuid } from '../../../stores/generateUuid';
import { Block } from '../Blocks';
import lodash from 'lodash';
import {SortableContext} from '@dnd-kit/sortable';

const item2block = (item) => {
    let type = `uuid-${item.type}`;
    return {
        type,
        uuid: generateUuid(type),
        ref: item.uuid,
        readonly: false,
    }
}

export const UUIDDrawer = ({itemType}) => {
    // itemType is the "type" of item from the EvdStore that the uuid corresponds to.
    // e.g. 'thingType', 'machine', 'waypoint', etc.
    const context = useStore(useCallback(state => lodash.filter(state.data, v=>v.type===itemType),[itemType]), shallow);

    const ancestors = [
        {uuid:'drawer',...acceptLookup.drawer.default}
    ];

    const entries = Object.values(context).map(item2block)
    console.log(entries)

    return (
        <SortableContext items={entries.map(item=>item.uuid)}>
            {entries.map(item=>(
                <Block ancestors={ancestors} staticData={item} context={context} dragDisabled={false}/>
            ))}
        </SortableContext>
    );
};
