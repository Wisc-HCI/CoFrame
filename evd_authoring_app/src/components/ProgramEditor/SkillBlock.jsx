import React, {forwardRef} from 'react';
import { Col, Row, Button, Dropdown, Menu } from 'antd';
import Icon, { EllipsisOutlined, UnlockOutlined, LockOutlined } from '@ant-design/icons';

import { ItemSortable } from './Wrappers';
import { NodeZone } from './NodeZone';
import useStore from '../../stores/Store';
import { acceptLookup } from './acceptLookup';
import blockStyles from './blockStyles';
import { ReactComponent as ContainerIcon } from '../CustomIcons/Container.svg'
import './highlight.css';

import { EditableTagGroup } from './Tags/EditableTagGroup';
// import { EditableTag } from './Tags/EditableTag';

export const SkillBlock = forwardRef(({style,data,ancestors,preview,context}, ref) => {

    const [frame,focusItem,setFocusItem,moveChildPrimitive] = useStore(state=>([state.frame,state.focusItem,state.setFocusItem,state.moveChildPrimitive]));
    const focused = focusItem.uuid === data.uuid;
    const toggleSkillEditable = useStore(state=>state.toggleSkillEditable);

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
        currentContext[arg.uuid] = {name:arg.name,real:false}
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
                <Col ref={ref} span={17} style={{backgroundColor:'rgba(255,255,255,0.1)',borderRadius:3,padding:4,cursor: "grab"}}>
                    <Icon style={{marginLeft:4}} component={ContainerIcon} />{' '}{data.name}
                </Col>
                <Col span={6} offset={1} style={{textAlign:'end'}}>
                    {editingEnabled ? <UnlockOutlined /> : <LockOutlined />}
                    <Dropdown overlay={
                        <Menu>
                            {!editingEnabled && <Menu.Item key='show' onClick={() => toggleSkillEditable(data.uuid)}>
                                Enable Editing
                            </Menu.Item>}
                            {editingEnabled && <Menu.Item key='show' onClick={() => toggleSkillEditable(data.uuid)}>
                                Disable Editing
                            </Menu.Item>}
                        </Menu>
                        }>
                        <Button
                            type='text'
                            style={{ marginLeft: 2 }}
                            icon={<EllipsisOutlined />}
                        />
                    </Dropdown>
                </Col>
            </Row>
            <Row>
                {!inDrawer && <EditableTagGroup skill={data} ancestors={skillAncestors}/>}
            </Row>
            <NodeZone
              ancestors={skillAncestors}
              style={{paddingTop:4,paddingBottom:4}}
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