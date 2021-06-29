import React, {forwardRef} from 'react';
import { Card, Button } from 'antd';
import { EllipsisOutlined } from '@ant-design/icons';
import { ItemSortable } from './Wrappers';
import useGuiStore from '../../stores/GuiStore';
import { acceptLookup } from './acceptLookup';

export const SkillBlock = forwardRef((props, ref) => {

    const setFocusItem = useGuiStore(state=>state.setFocusItem);

    const fieldData = acceptLookup['node.primitive.hierarchical.program.'].primitiveIds;

    const ancestors = [
        {uuid:props.data.uuid,...fieldData},
        ...props.ancestors
    ];

    const blockStyles = {
        display:'inline-block',
        transform:`translate3d(${props.data.transform.x}px,${props.data.transform.y}px,0)`
    };

    return (
        <div {...props} ref={ref} style={{...props.style,...blockStyles}}>
            <Card 
                title={<span><i><b>Macro </b></i>{props.data.name}</span>} 
                role="Box" 
                style={{minWidth:250}}
                headStyle={{backgroundColor:'#62869e'}}
                bodyStyle={{minHeight:30,padding:0}}
                extra={
                    <Button
                        style={{marginLeft:20}}
                        onClick={() => false && setFocusItem('program', props.data.uuid)}
                        icon={<EllipsisOutlined />}
                    />
                }
                >
                <div>
                    {props.data.primitiveIds.map((id,idx)=>(
                        <ItemSortable key={id} id={id} idx={idx} ancestors={ancestors} itemType='primitive'/>
                    ))}
                </div>
            </Card>
        </div>
    )
});