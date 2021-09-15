import React, {useCallback} from 'react';
import { Col, Row, Button, Dropdown, Menu } from 'antd';
import Icon, { EllipsisOutlined, UnlockOutlined, LockOutlined } from '@ant-design/icons';
import { SortableSeparator } from './SortableSeparator';
// import { ItemSortable } from './Wrappers';
import { NodeZone } from './NodeZone';
import useStore from '../../stores/Store';
import { acceptLookup } from './acceptLookup';
import blockStyles from './blockStyles';
import { ReactComponent as ContainerIcon } from '../CustomIcons/Container.svg'
import './highlight.css';
import { PrimitiveBlock } from './PrimitiveBlock';

import { EditableTagGroup } from './Tags/EditableTagGroup';
import { useDrag } from 'react-dnd';
// import { EditableTag } from './Tags/EditableTag';

export const SkillBlock = ({staticData,uuid,parentData,dragBehavior,ancestors,context}) => {

    const [frame,focusItem,setFocusItem,
        moveChildPrimitive,insertChildPrimitive] = useStore(state=>(
        [state.frame,state.focusItem,state.setFocusItem,
        state.moveChildPrimitive,state.insertChildPrimitive]));
    
    const data = useStore(useCallback((state)=>{
        return staticData ? staticData : state.data.skills[uuid];
    },[staticData,uuid]))
    
    const focused = focusItem.uuid === data.uuid;
    const toggleSkillEditable = useStore(state=>state.toggleSkillEditable);

    const fieldData = acceptLookup['node.primitive.hierarchical.skill.'].primitiveIds;

    const inDrawer = ancestors[0].uuid === 'drawer';
    const editingEnabled = !inDrawer && data.editable;

    const skillAncestors = [
        {uuid:data.uuid,...fieldData},
        ...ancestors
    ];

    // Code for handling the draggability of the skill node itself
    const [{ isDragging }, drag, preview] = useDrag(() => ({
        type: data.type,
        item: { ...data, parentData, dragBehavior},
        options: { dragEffect: dragBehavior },
        collect: monitor => ({
          isDragging: monitor.isDragging()
        })
    }))

    // Code for handling how primitives are handled when dropped in
    const primitiveDrop = (dropData, idx) => {
        if (dropData.parentData.uuid === uuid && dropData.dragBehavior === 'move') {
            const newIdx = dropData.idx <= idx ? idx - 1 : idx;
            if (newIdx === dropData.idx) {
                return
            }
            moveChildPrimitive(dropData.uuid, dropData.parentData.uuid, uuid, dropData.idx, newIdx);
        } else {
            insertChildPrimitive(dropData, uuid, idx);
        }
    }

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
        zIndex: focused ? 100 : 1,
        opacity: isDragging ? 0.4 : 1
    };

    return (
        <div ref={preview} style={dragBlockStyles} className={focused?`focus-${frame}`:null}>
            <Row style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
                <Col ref={drag} span={17} style={{backgroundColor:'rgba(255,255,255,0.1)',borderRadius:3,padding:4,cursor: "grab",zIndex:100}}>
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
                style={{ paddingTop: 4, paddingBottom: 4 }}
                ancestors={skillAncestors}
                onDrop={(dropData) => moveChildPrimitive(dropData, uuid, 'program', 0)}
                emptyMessage='No Actions'
                enabled={true}
            >
                {data.primitiveIds.map((id, idx) => (
                    <React.Fragment key={idx}>
                        {idx === 0 && (
                            <SortableSeparator
                                key={0}
                                height={60}
                                ancestors={skillAncestors}
                                context={currentContext}
                                onDrop={(dropData) => primitiveDrop(dropData, 0)}
                                dropDisabled={false}
                            />
                        )}
                        <PrimitiveBlock
                            key={id}
                            uuid={id}
                            parentData={{ type: 'program', uuid, field: 'primitive_uuids' }}
                            dragBehavior='move'
                            ancestors={skillAncestors}
                            context={currentContext}
                            idx={idx}
                            dropDisabled={true}
                            dragDisabled={false}
                            after={
                                <SortableSeparator
                                    ancestors={skillAncestors}
                                    height={60}
                                    context={currentContext}
                                    onDrop={(dropData) => primitiveDrop(dropData, idx + 1)}
                                    dropDisabled={false}
                                />
                            }
                        />
                    </React.Fragment>
                ))}
            </NodeZone>
        </div>
    )
};