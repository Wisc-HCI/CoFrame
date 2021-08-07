import React, {forwardRef} from 'react';
import { Col, Row, Button } from 'antd';
import Icon, { EllipsisOutlined, UnlockOutlined, LockOutlined } from '@ant-design/icons';
import { ItemSortable } from './Wrappers';
import useGuiStore from '../../stores/GuiStore';
import { acceptLookup } from './acceptLookup';
import blockStyles from './blockStyles';
import { ReactComponent as ContainerIcon } from '../CustomIcons/Container.svg'
import './highlight.css';

export const SkillBlock = forwardRef(({style,data,ancestors,preview}, ref) => {

    const [frame,focusItem,setFocusItem] = useGuiStore(state=>([state.frame,state.focusItem,state.setFocusItem]));
    const focused = focusItem.uuid === data.uuid;

    const fieldData = acceptLookup['node.primitive.hierarchical.skill.'].primitiveIds;

    const inDrawer = ancestors[0].uuid === 'drawer';
    const editingEnabled = !inDrawer && data.editable;

    const skillAncestors = [
        {uuid:data.uuid,...fieldData},
        ...ancestors
    ];

    const primitiveBinStyle = {
        backgroundColor: 'rgba(0,0,0,0.5)',
        borderRadius: 5,
        minWidth: 20,
        minHeight: 25,
        padding: 3,
      }

    const dragBlockStyles = {
        display:'inline-block',
        position: ancestors[0].uuid == 'drawer' ? 'relative' : 'absolute',
        // transform:`translate3d(${data.transform.x}px,${data.transform.y}px,0)`,
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

    console.log(`${data.name} ${data.transform.x} ${data.transform.y}`)

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
            <div style={primitiveBinStyle}>
                {data.primitiveIds.map((id,idx)=>(
                    <ItemSortable key={id} id={id} idx={idx} ancestors={skillAncestors} itemType='primitive'/>
                ))}
            </div>
        </div>
    )
});