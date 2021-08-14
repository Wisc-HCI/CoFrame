import React from 'react';
// import {
//     SortableContext,
//     verticalListSortingStrategy,
// } from '@dnd-kit/sortable';
import { GenericSortable } from './Wrappers';

import { primitiveTypes, fromPrimitiveTemplate } from '../../stores/templates';
import { acceptLookup } from './acceptLookup';

export const PrimitivesDrawer = (_) => {

    const ancestors = [
        {uuid:'drawer',...acceptLookup.drawer.default}
    ];

    return (
        <React.Fragment>
            {primitiveTypes.map((type)=>(
                <GenericSortable key={type} ancestors={ancestors} itemType='primitive' data={fromPrimitiveTemplate(type)}/>
            ))}
        </React.Fragment>
    );
};
