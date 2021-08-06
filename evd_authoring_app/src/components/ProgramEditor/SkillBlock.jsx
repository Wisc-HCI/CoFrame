import React, {forwardRef} from 'react';
import { Row, Button } from 'antd';
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
        transform:`translate3d(${data.transform.x}px,${data.transform.y}px,0)`,
        backgroundColor:
          blockStyles['node.primitive.hierarchical.skill.'],
        minHeight: 30,
        minWidth: 250,
        borderRadius: 3,
        margin: 4,
        padding: 5,
        position: 'relative',
        zIndex: focused ? 100 : 1
    };

    return (
        <div ref={preview} style={{...style,...dragBlockStyles}} className={focused?`focus-${frame}`:null}>
            <Row ref={ref} style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
                <span>
                    <Icon style={{marginLeft:4}} component={ContainerIcon} />{' '}{data.name}
                </span>
                <span style={{marginLeft:15}}>
                    {editingEnabled ? <UnlockOutlined /> : <LockOutlined />}
                    <Button
                        type='text'
                        style={{marginLeft:2}}
                        onClick={() => setFocusItem('skill', data.uuid)}
                        icon={<EllipsisOutlined />}
                    />
                </span>
            </Row>
            <div style={primitiveBinStyle}>
                {data.primitiveIds.map((id,idx)=>(
                    <ItemSortable key={id} id={id} idx={idx} ancestors={skillAncestors} itemType='primitive'/>
                ))}
            </div>
        </div>
    )
});