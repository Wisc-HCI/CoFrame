import React, { useState } from 'react';
import { Row, Button, Input } from 'antd';
import Icon, { UnlockOutlined, EyeOutlined, SaveOutlined, EditOutlined } from '@ant-design/icons';
import { useDrag } from 'react-dnd';
// import { ItemSortable } from './Wrappers';
import { ActionBlock } from './ActionBlock';
import { NodeZone } from './NodeZone';
import useStore from '../../stores/Store';
import shallow from 'zustand/shallow';
import blockStyles from './blockStyles';
import { ReactComponent as ContainerIcon } from '../CustomIcons/Container.svg'
import './highlight.css';

import { acceptLookup } from './acceptLookup';
import { SortableSeparator } from './SortableSeparator';

export const ProgramBlock = ({ parentData, dragBehavior, context, ancestors }) => {

    const [uuid, name, type, transform,
        primitiveIds, executable, moveChildPrimitive,
        deleteHierarchical, deleteChildPrimitive,
        insertChildPrimitive, setName] = useStore(state => ([
            state.uuid,
            state.name,
            state.type,
            state.transform,
            state.primitiveIds,
            state.executablePrimitives[state.uuid] ? true : false,
            state.moveChildPrimitive,
            state.deleteHierarchical,
            state.deleteChildPrimitive,
            state.insertChildPrimitive,
            state.setName
        ]), shallow)
    
    const [editing, setEditing] = useState(false);

    const data = { uuid, name, type, primitiveIds, transform };

    const fieldData = acceptLookup['node.primitive.hierarchical.program.'].primitiveIds;

    const programAncestors = [
        { uuid: uuid, ...fieldData },
        ...ancestors
    ];

    const [frame, focusItem, setFocusItem] = useStore(state => ([
        state.frame,
        state.focusItem,
        state.setFocusItem
    ]), shallow);

    const focused = focusItem.uuid === uuid;

    const [{ isDragging }, drag, preview] = useDrag(() => ({
        type: data.type,
        item: { ...data, parentData, dragBehavior },
        options: { dragEffect: dragBehavior },
        collect: monitor => ({
            isDragging: monitor.isDragging()
        })
    }))

    const primitiveDrop = (dropData, idx) => {
        if (dropData.parentData.uuid === uuid && dropData.dragBehavior === 'move') {
            const newIdx = dropData.idx <= idx ? idx - 1 : idx;
            if (newIdx === dropData.idx) {
                return
            }
            moveChildPrimitive(dropData.uuid, dropData.parentData.uuid, uuid, dropData.idx, newIdx);
        } else if (dropData.dragBehavior === 'move') {
            moveChildPrimitive(dropData.uuid, dropData.parentData.uuid, uuid, dropData.idx, idx)
        } else {
            insertChildPrimitive(dropData, uuid, idx);
        }
    }

    const onChildDelete = (dropData) => {
        if (dropData.type.includes('hierarchical')) {
          deleteHierarchical(dropData,data.uuid)
        } else {
          deleteChildPrimitive(data.uuid,dropData.uuid)
        }
      }

    const dragBlockStyles = {
        display: 'inline-block',
        position: 'absolute',
        left: transform.x,
        top: transform.y,
        opacity: isDragging ? 0.4 : 1,
        backgroundColor:
            blockStyles['node.primitive.hierarchical.program.'],
        minHeight: 30,
        minWidth: 250,
        borderRadius: 3,
        padding: 5,
        zIndex: focused ? 100 : 1
    };

    return (
        <div onDrag={e => e.stopPropagation()} ref={preview} style={dragBlockStyles} className={focused ? `focus-${frame}` : null}>
            <Row style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
                <Row ref={editing ? null : drag} wrap={false} align='middle' style={{ boxShadow: editing ? 'inset 0px 0px 2px 1px #ffffff' : null, backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, textAlign: 'start', flex: 1, minWidth: 130, maxWidth: 200, cursor: editing ? null : "grab", zIndex: 101, marginRight: 5, height: 32 }}>
                    <Icon style={{ marginLeft: 4 }} component={ContainerIcon} />
                    <Input style={{ maxWidth: 200, color: 'white', cursor: editing ? 'text' : "grab" }} bordered={false} disabled={!editing} value={name} onChange={(e) => setName(e.target.value)} />
                </Row>
                <Row wrap={false} align="middle" style={{ textAlign: 'end' }}>
                    <Button type='text' onClick={() => setEditing(!editing)} icon={editing ? <SaveOutlined/> : <EditOutlined/>}/>
                    {executable && <Button type='text' icon={<EyeOutlined/>} onClick={(e) => {e.stopPropagation();setFocusItem('program', uuid)}}/>}
                    <UnlockOutlined style={{marginRight:5,marginLeft:5}}/>
                    {/* <Button
                        type='text'
                        style={{ marginLeft: 2 }}
                        onClick={(e) => { e.stopPropagation(); setFocusItem('program', uuid) }}
                        icon={<EllipsisOutlined />}
                    /> */}
                </Row>
            </Row>
            <NodeZone
                ancestors={programAncestors}
                onDrop={(dropData) => primitiveDrop(dropData, 0)}
                emptyMessage='No Actions'
                context={context}
            >
                {primitiveIds.map((id, idx) => (
                    <React.Fragment key={idx}>
                        {idx === 0 && (
                            <SortableSeparator
                                key={0}
                                spacing={idx === 0 ? 0 : 5}
                                height={30}
                                ancestors={programAncestors}
                                context={context}
                                onDrop={(dropData) => primitiveDrop(dropData, 0)}
                                dropDisabled={false}
                            />
                        )}
                        <ActionBlock
                            key={id}
                            uuid={id}
                            parentData={{ type: 'program', uuid, field: 'primitive_uuids' }}
                            dragBehavior='move'
                            ancestors={programAncestors}
                            context={context}
                            idx={idx}
                            onDelete={onChildDelete}
                            dragDisabled={false}
                            after={
                                <SortableSeparator
                                    ancestors={programAncestors}
                                    height={30}
                                    end={idx === primitiveIds.length - 1}
                                    spacing={idx === primitiveIds.length - 1 ? 0 : 5}
                                    context={context}
                                    onDrop={(dropData) => primitiveDrop(dropData, idx + 1)}
                                    dropDisabled={false}
                                />
                            }
                        />
                    </React.Fragment>
                ))}
            </NodeZone>
        </div>
    );
};
