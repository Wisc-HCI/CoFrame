import React from 'react';
import { Card, Button } from 'antd';
import { EllipsisOutlined } from '@ant-design/icons';
import {
    SortableContext,
    verticalListSortingStrategy,
} from '@dnd-kit/sortable';

import { ItemSortable } from './Wrappers';

import useEvdStore from '../../stores/EvdStore';
import useGuiStore from '../../stores/GuiStore';

export const ProgramBlock = (_) => {
    
    const {name,uuid,primitiveIds} = useEvdStore(state=>({
        name:state.name,
        uuid:state.uuid,
        primitiveIds:state.primitiveIds,
    }))

    const [dragItem,setFocusItem] = useGuiStore(state=>([
        state.dragItem,
        state.setFocusItem
    ]));

    return (
        <div style={{display:'inline-block'}}>
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
                }>
                <SortableContext items={primitiveIds} strategy={verticalListSortingStrategy}>
                    {primitiveIds.map(id=>(
                        <ItemSortable key={id} id={id} source={uuid} itemType='primitive' hide={dragItem!==null&&dragItem.uuid===id}/>
                    ))}
                </SortableContext>
            </Card>
        </div>
        
        
    );
};
