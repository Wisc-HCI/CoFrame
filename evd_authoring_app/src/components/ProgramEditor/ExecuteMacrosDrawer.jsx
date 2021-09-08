import React, {useCallback, useRef} from 'react';
// import {
//     SortableContext,
//     verticalListSortingStrategy,
// } from '@dnd-kit/sortable';
import { GenericSortable } from './Wrappers';

import { acceptLookup } from './acceptLookup';
import useEvdStore, {typeToKey} from '../../stores/EvdStore';

export const ExecuteMacrosDrawer = (_) => {

    const ancestors = [
        {uuid:'drawer',...acceptLookup.drawer.default}
    ];

    const data = useEvdStore(useCallback(state=>state.data[typeToKey('skill')]));
    console.log(data);

    return (
        <React.Fragment>
            {Object.entries(data).filter(obj=>!(obj[1]===undefined)).map((obj)=>(
                <GenericSortable key={obj[1].uuid} ancestors={ancestors} itemType='executeskill' data={obj[1]}/>
            ))}
        </React.Fragment>
    );
};
