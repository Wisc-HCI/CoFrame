import React from 'react';
import {
    SortableContext,
    verticalListSortingStrategy,
} from '@dnd-kit/sortable';
import { GenericSortable } from './Wrappers';

import { primitiveTypes, fromTemplate } from '../../stores/templates';
import { acceptLookup } from './acceptLookup';

export const PrimitivesDrawer = (_) => {

    const ancestors = [
        {uuid:'drawer',...acceptLookup.drawer.default}
    ];

    return (
        <SortableContext items={primitiveTypes} strategy={verticalListSortingStrategy}>
            {primitiveTypes.map((type,idx)=>(
                <GenericSortable key={type} idx={idx} ancestors={ancestors} source='drawer' itemType='primitive' data={fromTemplate(type)} hide={false}/>
            ))}
        </SortableContext>
    );
};
