import React from 'react';
// import {
//     SortableContext,
//     verticalListSortingStrategy,
// } from '@dnd-kit/sortable';
// import { GenericSortable } from './Wrappers';

import { primitiveTypes, fromPrimitiveTemplate } from '../../stores/templates';
import { acceptLookup } from './acceptLookup';
import { ActionBlock } from './ActionBlock';

export const PrimitivesDrawer = (_) => {

    const ancestors = [
        {uuid:'drawer',...acceptLookup.drawer.default}
    ];

    return (
        <React.Fragment>
            {primitiveTypes.map((type,i)=>(
                <div key={type} style={{paddingTop:5}} >
                    <ActionBlock
                        ancestors={ancestors} 
                        idx={i}
                        parentData={{type:'drawer',uuid:'drawer'}}
                        staticData={fromPrimitiveTemplate(type)} 
                        dragBehavior='copy'
                        dropDisabled
                        context={{}}/>
                </div>
            ))}
        </React.Fragment>
    );
};
