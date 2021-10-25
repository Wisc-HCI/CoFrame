import React from 'react';
import { primitiveTypes, fromPrimitiveTemplate } from '../../../stores/templates';
import { acceptLookup } from '../acceptLookup';
import {SortableContext} from '@dnd-kit/sortable';
import { Block } from '../Blocks';

export const PrimitivesDrawer = (_) => {

    const ancestors = [
        { uuid: 'drawer', ...acceptLookup.drawer.default }
    ];

    const entries = primitiveTypes.map(type=>fromPrimitiveTemplate(type))

    return (
        <SortableContext items={entries.map(item=>item.uuid)} id={'primitives-drawer'}>
            {entries.map(item=>(
                <Block ancestors={ancestors} staticData={item} context={{}} dragDisabled={false}/>
            ))}
        </SortableContext>
    );
};