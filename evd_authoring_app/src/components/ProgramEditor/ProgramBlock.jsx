import React from 'react';
import { Col, Row, Button } from 'antd';
import Icon, { EllipsisOutlined, UnlockOutlined } from '@ant-design/icons';
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

export const ProgramBlock = ({parentData,dragBehavior,context,ancestors}) => {

    const [uuid, name, type, transform,
        primitiveIds, moveChildPrimitive,
        insertChildPrimitive] = useStore(state => ([
            state.uuid,
            state.name,
            state.type,
            state.transform,
            state.primitiveIds,
            state.moveChildPrimitive,
            state.insertChildPrimitive
        ]),shallow)

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
    ]),shallow);

    const focused = focusItem.uuid === uuid;

    const [{ isDragging }, drag, preview] = useDrag(() => ({
        type: data.type,
        item: { ...data, parentData, dragBehavior},
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
        <div onDrag={e=>e.stopPropagation()} ref={preview} style={dragBlockStyles} className={focused ? `focus-${frame}` : null}>
            <Row style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
                <Col ref={drag} span={17} style={{ backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, cursor: 'grab' ,zIndex:1000}}>
                    <Icon style={{ marginLeft: 4 }} component={ContainerIcon} />{' '}{name}
                </Col>
                <Col span={6} offset={1} style={{ textAlign: 'end' }}>
                    <UnlockOutlined />
                    <Button
                        type='text'
                        style={{ marginLeft: 2 }}
                        onClick={(e) => { e.stopPropagation(); setFocusItem('program', uuid) }}
                        icon={<EllipsisOutlined />}
                    />
                </Col>
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
                                spacing={idx===0 ? 0 : 5}
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
                            dragDisabled={false}
                            after={
                                <SortableSeparator
                                    ancestors={programAncestors}
                                    height={30}
                                    end={idx === primitiveIds.length-1}
                                    spacing={idx ===primitiveIds.length-1 ? 0 : 5}
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
