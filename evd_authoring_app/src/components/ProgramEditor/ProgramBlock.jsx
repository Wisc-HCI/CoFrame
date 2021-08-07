import React from 'react';
import { Col, Row, Button } from 'antd';
import Icon, { EllipsisOutlined, UnlockOutlined } from '@ant-design/icons';
import { useDrag } from 'react-dnd';
import { ItemSortable } from './Wrappers';
import { NodeZone } from './NodeZone';
import useGuiStore from '../../stores/GuiStore';
import useEvdStore from '../../stores/EvdStore';
import blockStyles from './blockStyles';
import { ReactComponent as ContainerIcon } from '../CustomIcons/Container.svg'
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

    const moveChildPrimitive = useEvdStore(state=>state.moveChildPrimitive);

    const data = {uuid,name,type,primitiveIds,transform};

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
    const [{isDragging}, drag, preview] = useDrag({
        type: data.type,
        item: data,
        collect: monitor => ({
          isDragging: monitor.isDragging()
        })
    })

    const dragBlockStyles = {
        display:'inline-block',
        position:'absolute',
        left:transform.x,
        top:transform.y,
        opacity: isDragging ? 0.4 : 1,
        backgroundColor:
          blockStyles['node.primitive.hierarchical.program.'],
        minHeight: 30,
        minWidth: 250,
        borderRadius: 3,
        margin: 4,
        padding: 5,
        zIndex: focused ? 100 : 1
    };

    return (
        <div ref={preview} {...props} style={dragBlockStyles} className={focused?`focus-${frame}`:null}>
            <Row style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
                <Col ref={drag} span={17} style={{backgroundColor:'rgba(255,255,255,0.1)',borderRadius:3,padding:4}}>
                    <Icon style={{marginLeft:4}} component={ContainerIcon} />{' '}{name}
                </Col>
                <Col span={6} offset={1}>
                    <UnlockOutlined/>
                    <Button
                        type='text'
                        style={{marginLeft:2}}
                        onClick={(e) => {e.stopPropagation();setFocusItem('program', uuid)}}
                        icon={<EllipsisOutlined />}
                    />
                </Col>
            </Row>
            <NodeZone
              ancestors={ancestors}
              onDrop={(data) => moveChildPrimitive(data,uuid,0)}
              emptyMessage='No Actions'
              enabled={true}
            >
                {primitiveIds.map((id,idx)=>(
                    <ItemSortable key={id} id={id} idx={idx} ancestors={ancestors} itemType='primitive'/>
                ))}
            </NodeZone>
        </div>
    );
};
