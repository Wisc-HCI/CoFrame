import React from 'react';
import { Card, Button } from 'antd';
import { EllipsisOutlined } from '@ant-design/icons';
import { useDrag } from 'react-dnd';
import { ItemSortable } from './Wrappers';
import useGuiStore from '../../stores/GuiStore';
import useEvdStore from '../../stores/EvdStore';
import blockStyles from './blockStyles';
import './highlight.css';

import { acceptLookup } from './acceptLookup';

export const ProgramBlock = (props) => {

    const [uuid, name, type, transform, primitiveIds] = useEvdStore(state=>([
        state.uuid,
        state.name,
        state.type,
        state.transform,
        state.primitiveIds
    ]))

    const data = {uuid,name,type,primitiveIds};

    const fieldData = acceptLookup['node.primitive.hierarchical.program.'].primitiveIds;

    const ancestors = [
        {uuid:uuid,...fieldData},
        ...props.ancestors
    ];

    const [frame,focusItem,setFocusItem] = useGuiStore(state=>([
        state.frame,
        state.focusItem,
        state.setFocusItem
    ]));

    const focused = focusItem.uuid === uuid;

    // Code for handling the draggability of the program node itself
    const [{isDragging}, drag] = useDrag({
        type: data.type,
        item: data,
        collect: monitor => ({
          isDragging: monitor.isDragging()
        })
    })

    const dragBlockStyles = {
        display:'inline-block',
        transform:`translate3d(${transform.x}px,${transform.y}px,0)`,
        opacity: isDragging ? 0.4 : 1,
    };

    return (
        <div ref={drag} {...props} style={dragBlockStyles} className={focused?`focus-${frame}`:null}>
            <Card 
                title={name} 
                role="Box" 
                style={{minWidth:250}}
                headStyle={{backgroundColor:blockStyles['node.primitive.hierarchical.program.']}}
                bodyStyle={{minHeight:30,padding:0}}
                className={focusItem.type==='program'&&`focus-${frame}`}
                extra={
                    <Button
                        style={{marginLeft:20}}
                        onClick={() => false && setFocusItem('program', uuid)}
                        icon={<EllipsisOutlined />}
                    />
                }
                >
                <div>
                    {primitiveIds.map((id,idx)=>(
                        <ItemSortable key={id} id={id} idx={idx} ancestors={ancestors} itemType='primitive'/>
                    ))}
                </div>
            </Card>
        </div>
    );
};
