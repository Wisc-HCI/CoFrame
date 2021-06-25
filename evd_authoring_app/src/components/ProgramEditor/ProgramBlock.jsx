import React, {forwardRef} from 'react';
import { Card, Button } from 'antd';
import { EllipsisOutlined } from '@ant-design/icons';
import {
    SortableContext,
    verticalListSortingStrategy
} from '@dnd-kit/sortable';

import { useDraggable } from "@dnd-kit/core";

import { ItemSortable } from './Wrappers';

import useEvdStore from '../../stores/EvdStore';
import useGuiStore from '../../stores/GuiStore';

import { acceptLookup } from './acceptLookup';
import {CSS} from '@dnd-kit/utilities';



export const ProgramBlock = forwardRef((props, ref) => {

    const {name,uuid,primitiveIds} = useEvdStore(state=>({
        name:state.name,
        uuid:state.uuid,
        primitiveIds:state.primitiveIds,
    }))

    const ancestors = [
        {uuid:uuid,...acceptLookup['node.primitive.hierarchical.program'].primitiveIds},
        {uuid:'grid',...acceptLookup.grid.primitiveIds}
    ];

    const [dragItem,setFocusItem] = useGuiStore(state=>([
        state.dragItem,
        state.setFocusItem
    ]));

    const styles = {display:'inline-block'};





    return (
        <div {...props} ref={ref} style={{...props.style, ...styles}} >
            <Card
                title={name}
                role="Box"
                style={{minWidth:250}}
                headStyle={{backgroundColor:'#1f1f1f'}}
                bodyStyle={{minHeight:30,padding:0}}
                extra={
                    <Button
                        style={{marginLeft:20}}
                        onClick={() => false && setFocusItem('program', uuid)}
                        icon={<EllipsisOutlined />}
                    />
                }
                >
                <SortableContext items={primitiveIds} strategy={verticalListSortingStrategy}>
                    {primitiveIds.map((id,idx)=>(
                        <ItemSortable key={id} id={id} idx={idx} ancestors={ancestors} itemType='primitive' hide={dragItem!==null&&dragItem.uuid===id}/>
                    ))}
                </SortableContext>
            </Card>
        </div>
    );
});
