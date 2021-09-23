import React, { useCallback, useState } from 'react';
import { Badge, Button, Row, Input } from 'antd';
import Icon, { RightOutlined, EditOutlined, SaveOutlined } from '@ant-design/icons';
import { useDrag } from 'react-dnd';
import { SortableSeparator } from './SortableSeparator';
import { NodeZone } from './NodeZone';
import useStore from '../../stores/Store';
import shallow from 'zustand/shallow';
import blockStyles from './blockStyles';
import { ReactComponent as ContainerIcon } from '../CustomIcons/Container.svg';
import { acceptLookup } from './acceptLookup';
import './highlight.css';
import { ActionBlock } from './ActionBlock';
import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';
import useMeasure from 'react-use-measure';

export const HierarchicalBlock = ({ staticData, uuid, parentData, dragBehavior, dragDisabled, ancestors, context, onDelete, idx, after, locked }) => {

  const [frame, focusItem, setItemProperty, deleteHierarchical,
    moveChildPrimitive, insertChildPrimitive] = useStore(state => (
      [state.frame, state.focusItem, state.setItemProperty, state.deleteHierarchical,
      state.moveChildPrimitive, state.insertChildPrimitive]),shallow);

  const data = useStore(useCallback((state) => {
    return staticData ? staticData : state.data.primitives[uuid];
  }, [staticData, uuid]))

  const [editing, setEditing] = useState(false);
  const [expanded, setExpanded] = useState(true);
  const [nodeListRef, { height }] = useMeasure();
  const nodeListStyle = useSpring({ height: height, config: config.stiff });

  const focused = focusItem.uuid === data.uuid;

  const inDrawer = ancestors[0].uuid === 'drawer';
  const editingEnabled = !inDrawer && data.editable;

  // Code for handling the draggability of the skill node itself
  const [{ isDragging }, drag, preview] = useDrag(() => ({
    type: data.type,
    item: { ...data, parentData, dragBehavior, onDelete: onDelete ? onDelete : ()=>deleteHierarchical(data,ancestors[0].uuid), idx },
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
    } else if (dropData.dragBehavior === 'move') {
      moveChildPrimitive(dropData.uuid, dropData.parentData.uuid, uuid, dropData.idx, idx)
    } else {
      insertChildPrimitive(dropData, uuid, idx);
    }
  }

  const fieldData = acceptLookup['node.primitive.hierarchical.'].primitiveIds;

  const primitiveAncestors = [
    { uuid: data.uuid, ...fieldData },
    ...ancestors
  ]

  const dragBlockStyles = {
    backgroundColor:
      blockStyles[data.type],
    minHeight: 30,
    minWidth: 200,
    borderRadius: 3,
    fontSize: 14,
    padding: 5,
    position: 'relative',
    margin: 0,
    textAlign: 'left',
    zIndex: focused ? 100 : 1
  };

  return (
    <div hidden={isDragging && dragBehavior === 'move'}>
      <div ref={preview} style={dragBlockStyles} className={focused ? `focus-${frame}` : null}>
        <Row wrap={false} style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
          <Row ref={editing ? null : drag} wrap={false} align='middle' style={{ boxShadow: editing?'inset 0px 0px 2px 1px #ffffff':null, borderColor: 'white', backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, textAlign: 'start', flex: 1, minWidth: 130, maxWidth: 200, cursor: dragDisabled ? "not-allowed" : "grab",zIndex:101, marginRight:5, height: 32 }}>
            <Icon style={{ marginLeft: 4 }} component={ContainerIcon} />
            <Input style={{maxWidth: 200, color:'white',cursor: editing ? 'text' : dragDisabled ? "not-allowed" : "grab"}} bordered={false} disabled={!editing} value={data.name} onChange={(e)=>setItemProperty('primitive', data.uuid, 'name', e.target.value)}/> 
          </Row>
          <Row wrap={false} align='middle' style={{ textAlign: 'end' }}>
            {!inDrawer && <Button type='text' onClick={() => setExpanded(!expanded)} icon={<RightOutlined rotate={expanded ? 90 : 0} />} style={{zIndex:200}}/>}
            {editingEnabled && <Button type='text' onClick={() => setEditing(!editing)} icon={editing ? <SaveOutlined/> : <EditOutlined/>}/>}
            <Badge count={data.primitiveIds.length} showZero={true} style={{backgroundColor:'rgba(0,0,0,0.3)',marginRight:5, marginLeft:5}}/>
          </Row>
        </Row>
        <animated.div style={nodeListStyle}>
          <div ref={nodeListRef}>
            {expanded && 
            <NodeZone
              ancestors={primitiveAncestors}
              onDrop={(dropData) => primitiveDrop(dropData, 0)}
              emptyMessage='No Actions'
              dropDisabled={!editingEnabled || isDragging || locked}
              context={context}
            >
              {data.primitiveIds.map((id, idx) => (
                <React.Fragment key={idx}>
                  {idx === 0 && (
                    <SortableSeparator
                      key={0}
                      spacing={idx === 0 ? 0 : 5}
                      height={40}
                      ancestors={primitiveAncestors}
                      context={context}
                      onDrop={(dropData) => primitiveDrop(dropData, 0)}
                      dropDisabled={!editingEnabled || isDragging || locked}
                    />
                  )}
                  <ActionBlock
                    key={id}
                    uuid={id}
                    parentData={{ type: 'primitive', uuid, field: 'primitive_uuids' }}
                    dragBehavior='move'
                    ancestors={primitiveAncestors}
                    context={context}
                    idx={idx}
                    dragDisabled={!editingEnabled || isDragging || locked}
                    after={
                      <SortableSeparator
                        ancestors={primitiveAncestors}
                        height={40}
                        end={idx === data.primitiveIds.length - 1}
                        spacing={idx === data.primitiveIds.length - 1 ? 0 : 5}
                        context={context}
                        onDrop={(dropData) => primitiveDrop(dropData, idx + 1)}
                        dropDisabled={!editingEnabled || isDragging || locked}
                      />
                    }
                  />
                </React.Fragment>
              ))}
            </NodeZone>  
          }
          </div>
        </animated.div>
      </div>
      {after}
    </div>

  )
};