import React, {forwardRef} from 'react';
import { Col, Row, Button } from 'antd';
import Icon, { EllipsisOutlined, UnlockOutlined, LockOutlined } from '@ant-design/icons';
import { ItemSortable } from './Wrappers';
import { NodeZone } from './NodeZone';
import useStore from '../../stores/Store';
import { acceptLookup } from './acceptLookup';
import blockStyles from './blockStyles';
import { ReactComponent as ContainerIcon } from '../CustomIcons/Container.svg'
import './highlight.css';

export const SkillBlock = forwardRef(({style,data,ancestors,preview,context}, ref) => {

    const [frame,focusItem,setFocusItem,moveChildPrimitive] = useStore(state=>([state.frame,state.focusItem,state.setFocusItem,state.moveChildPrimitive]));
    const focused = focusItem.uuid === data.uuid;

    const fieldData = acceptLookup['node.primitive.hierarchical.skill.'].primitiveIds;

    const inDrawer = ancestors[0].uuid === 'drawer';
    const editingEnabled = !inDrawer && data.editable;

    const skillAncestors = [
        {uuid:data.uuid,...fieldData},
        ...ancestors
    ];

    // Extend the current context with any arg-based values
    let currentContext = {
        ...context
    };
    data.arguments.forEach(arg=>{
        currentContext[arg.uuid] = arg.name
    })

    const dragBlockStyles = {
        display:'inline-block',
        position: ancestors[0].uuid === 'drawer' ? 'relative' : 'absolute',
        left:data.transform.x,
        top:data.transform.y,
        backgroundColor:
          blockStyles['node.primitive.hierarchical.skill.'],
        minHeight: 30,
        minWidth: 250,
        borderRadius: 3,
        margin: 4,
        padding: 5,
        zIndex: focused ? 100 : 1
    };

    return (
        <div ref={preview} style={{...style,...dragBlockStyles}} className={focused?`focus-${frame}`:null}>
            <Row style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
                <Col ref={ref} span={17} style={{backgroundColor:'rgba(255,255,255,0.1)',borderRadius:3,padding:4}}>
                    <Icon style={{marginLeft:4}} component={ContainerIcon} />{' '}{data.name}
                </Col>
                <Col span={6} offset={1}>
                    {editingEnabled ? <UnlockOutlined /> : <LockOutlined />}
                    <Button
                        type='text'
                        style={{marginLeft:2}}
                        onClick={(e) => {e.stopPropagation();setFocusItem('skill', data.uuid)}}
                        icon={<EllipsisOutlined />}
                    />
                </Col>
            </Row>
            <NodeZone
              ancestors={skillAncestors}
              onDrop={(dropData) => moveChildPrimitive(dropData,data.uuid,'skill',0)}
              emptyMessage='No Actions'
              enabled={true}
            >
                {data.primitiveIds.map((id,idx)=>(
                    <ItemSortable 
                        key={id} 
                        id={id} 
                        idx={idx} 
                        ancestors={skillAncestors} 
                        itemType='primitive' 
                        context={currentContext} 
                        onMove={(dropData)=>moveChildPrimitive(dropData,data.uuid,'skill',idx)}
                        disabled={!editingEnabled}
                    />
                ))}
            </NodeZone>
        </div>
    )
});