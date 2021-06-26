import React, {forwardRef} from 'react';
import { Card, Button } from 'antd';
import { EllipsisOutlined, DragOutlined } from '@ant-design/icons';
import {
    SortableContext,
    verticalListSortingStrategy
} from '@dnd-kit/sortable';

import { useDraggable } from "@dnd-kit/core";

import { ItemSortable } from './Wrappers';
import useGuiStore from '../../stores/GuiStore';

import { acceptLookup } from './acceptLookup';
import {CSS} from '@dnd-kit/utilities';

export const ProgramBlock = forwardRef((props,ref) => {

    const {name,uuid,primitiveIds} = props.data;

    const ancestors = [
        {uuid:uuid,...acceptLookup['node.primitive.hierarchical.program'].primitiveIds},
        ...props.ancestors
    ];

    const [dragItem,setFocusItem] = useGuiStore(state=>([
        state.dragItem,
        state.setFocusItem
    ]));

    const styles = {
        display:'inline-block'
      };





    return (
        <div {...props} style={{...props.style, ...styles}}>
            <Card 
                title={<><Button type='text'  ref={ref} icon={<DragOutlined/>} style={{marginRight:10}}/>{name}</>} 
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
