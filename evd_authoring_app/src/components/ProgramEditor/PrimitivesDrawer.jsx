import React from 'react';
import {
    SortableContext,
    verticalListSortingStrategy,
} from '@dnd-kit/sortable';
import { GenericSortable } from './Wrappers';

import { primitiveTypes } from '../../stores/templates';

export const PrimitivesDrawer = (_) => {
    
    return (
        <SortableContext items={primitiveTypes} strategy={verticalListSortingStrategy}>
            {primitiveTypes.map(type=>(
                <GenericSortable key={type} type={type} source='drawer' itemType='primitive' hide={false}/>
            ))}
        </SortableContext>
    );
};
