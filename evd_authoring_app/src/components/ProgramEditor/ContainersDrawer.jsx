import React from 'react';
// import {
//     SortableContext,
//     verticalListSortingStrategy,
// } from '@dnd-kit/sortable';
import { GenericSortable } from './Wrappers';

import { containerTypes, fromContainerTemplate } from '../../stores/templates';
import { acceptLookup } from './acceptLookup';

export const ContainersDrawer = (_) => {

    const ancestors = [
        {uuid:'drawer',...acceptLookup.drawer.default}
    ];

    const itemTypes = ['trajectory','skill','primitive']; // These correspond to the types of the items in the templates

    return (
        <React.Fragment>
            {containerTypes.map((type,i)=>(
                <GenericSortable key={type} ancestors={ancestors} itemType={itemTypes[i]} data={fromContainerTemplate(type)}/>
            ))}
        </React.Fragment>
    );
};
