import React from 'react';
import { fromContainerTemplate } from '../../../stores/templates';
import { acceptLookup } from '../acceptLookup';
import {SortableContext} from '@dnd-kit/sortable';
import { Block } from '../Blocks';

export const ContainersDrawer = (_) => {

    const ancestors = [
        { uuid: 'drawer', ...acceptLookup.drawer.default }
    ];

    const entries = [
        fromContainerTemplate('trajectory'),
        fromContainerTemplate('skill'),
        fromContainerTemplate('hierarcical')
    ]

    return (
        <SortableContext items={entries.map(item=>item.uuid)} id={'containers-drawer'}>
            {entries.map(item=>(
                <Block key={item.uuid} ancestors={ancestors} staticData={item} context={{}} dragDisabled={false}/>
            ))}
        </SortableContext>
    );
};
