import React, {forwardRef} from 'react';
import { Card, Button } from 'antd';
import { EllipsisOutlined } from '@ant-design/icons';
import { ItemSortable } from './Wrappers';
import useGuiStore from '../../stores/GuiStore';
import { acceptLookup } from './acceptLookup';
import blockStyles from './blockStyles';
import './highlight.css';

export const SkillBlock = forwardRef((props, ref) => {

    const [frame,focusItem,setFocusItem] = useGuiStore(state=>([state.frame,state.focusItem,state.setFocusItem]));
    const focused = focusItem.uuid === props.data.uuid;

    const fieldData = acceptLookup['node.primitive.hierarchical.program.'].primitiveIds;

    const ancestors = [
        {uuid:props.data.uuid,...fieldData},
        ...props.ancestors
    ];

    const dragBlockStyles = {
        display:'inline-block',
        transform:`translate3d(${props.data.transform.x}px,${props.data.transform.y}px,0)`
    };

    return (
        <div {...props} ref={ref} style={{...props.style,...dragBlockStyles}} className={focused?`focus-${frame}`:null}>
            <Card 
                title={<span><i><b>Macro </b></i>{props.data.name}</span>} 
                role="Box" 
                style={{minWidth:250}}
                headStyle={{backgroundColor:blockStyles['node.primitive.hierarchical.skill.']}}
                bodyStyle={{minHeight:30,padding:0,position:'relative'}}
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