import React from 'react';
import { useDrag } from 'react-dnd';
import { Card, Button } from 'antd';
import { EllipsisOutlined } from '@ant-design/icons';

import useEvdStore from '../../stores/EvdStore';
import useGuiStore from '../../stores/GuiStore';

const style = {
    position: 'absolute',
    border: '1px solid gray',
    borderRadius: 3,
    color: 'black',
    backgroundColor: 'white',
    padding: '0.5rem 1rem',
    cursor: 'move',
};

export const ProgramBlock = (_) => {
    
    const {name,uuid,transform,primitiveIds,setName} = useEvdStore(state=>({
        name:state.name,
        uuid:state.uuid,
        transform:state.transform,
        primitiveIds:state.primitiveIds,
        setName:state.setName,
        setTransform:state.setTransform
    }))

    const [setFocusItem] = useGuiStore(state=>([state.setFocusItem]));
    
    const [{ isDragging }, drag] = useDrag(() => ({
        type: 'program',
        item: { id:uuid, left:transform.left, top:transform.top },
        collect: (monitor) => ({
            isDragging: monitor.isDragging(),
        }),
    }), [uuid, transform]);
    
    if (isDragging) {
        return <div ref={drag} />;
    }

    return (
        <div ref={drag} style={{cursor:'move',position:'absolute',left:transform.left, top:transform.top}} role='Box'>
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
                <div style={{height:'100%',width:'100%'}}>
                    {primitiveIds.map(id=>(
                        <div style={{backgroundColor:'#1f1f1f',borderRadius:3,margin:4}}>{id}</div>
                    ))}
                </div>
            </Card>
        </div>
        
    );
};
