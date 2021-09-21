import React, { useState } from "react";
import { Button, Dropdown, Input, Menu, Row } from "antd";
import { useDrag, useDrop } from 'react-dnd';
import Icon, { UnlockOutlined, LockOutlined, EllipsisOutlined, DeleteOutlined, EyeOutlined, EditOutlined } from '@ant-design/icons';
import useStore from "../../stores/Store";
import shallow from 'zustand/shallow';
import blockStyles from "./blockStyles";
import { ReactComponent as LocationIcon } from '../CustomIcons/Location.svg';
import { ReactComponent as MachineIcon } from '../CustomIcons/Gear.svg';
import { ReactComponent as ThingIcon } from '../CustomIcons/Thing.svg';
import { ReactComponent as WaypointIcon } from '../CustomIcons/Waypoint.svg';
import { ReactComponent as ContainerIcon } from '../CustomIcons/Container.svg';
import './highlight.css';

const ICONS = {
  'machine': MachineIcon,
  'location': LocationIcon,
  'placeholder': ThingIcon,
  'thing': ThingIcon,
  'waypoint': WaypointIcon,
  'trajectory': ContainerIcon
}

const validDrop = (item, ancestors) => {
  if (!ancestors[0].accepts.some(type => type === item.type)) {
    return false
  } else if (!ancestors.some(ancestor => ancestor.uuid === item.parentData.uuid) && item.parentData.type !== 'drawer') {
    return false
  }
  return true
}

export const UUIDBlock = ({
  data,
  idx,
  ancestors,
  context,
  onDelete,
  onDrop,
  onNameChange,
  dragDisabled,
  dropDisabled,
  hoverBehavior,
  dragBehavior,
  parentData,
  after
}) => {

  // props constains data,
  // which contains fields 'itemType' and 'uuid'
  // console.log(data)
  // hoverBehavior is either 'replace' or 'insert'
  const { itemType, uuid } = data;

  // Variables for Input field
  // const [editing, setEditing] = useState(data.name === '');
  // let editInputValue;
  // let saveEditInputRef;

  const [{ isDragging }, drag, preview] = useDrag(() => ({
    type: data.type,
    item: { ...data, parentData, dragBehavior, idx, onDelete },
    options: { dragEffect: dragBehavior },
    canDrag: _ => !dragDisabled,
    collect: monitor => ({
      isDragging: monitor.isDragging()
    })
  }))

  const [editing, setEditing] = useState(false);

  // We only care about the second value returned from useDrop (hence the [1] at the end)
  const [{ isOver, dragItem }, drop] = useDrop({
    accept: ancestors[0].accepts,
    drop: (otherItem, _) => {
      onDrop(otherItem)
    },
    canDrop: (otherItem, _) => {
      if (dropDisabled) {
        return false
      } else return validDrop(otherItem, ancestors);
    },
    collect: monitor => ({
      isOver: monitor.isOver(),
      dragItem: monitor.getItem()
    })
  })

  const [
    frame, focusItem,
    setFocusItem, clearFocusItem] = useStore(state => ([
      state.frame, state.focusItem,
      state.setFocusItem, state.clearFocusItem]
    ),shallow);
  const focused = focusItem.uuid === uuid;

  const inDrawer = ancestors[0].uuid === 'drawer';
  const isReal = context[uuid]?.real;
  const showMore = isReal || onNameChange;

  const blockStyle = {
    backgroundColor:
      blockStyles[itemType === 'placeholder' ? 'thing' : itemType],
    height: 43,
    width: '100%',
    borderRadius: 3,
    padding: 5,
    position: 'relative',
    zIndex: focused ? 100 : 1,
    opacity: isOver && hoverBehavior === 'replace' && dragItem ? 0.5 : 1
  };

  const displayData = !dropDisabled && isOver && hoverBehavior === 'replace' && dragItem && validDrop(dragItem, ancestors) ? dragItem : data

  return (
    <React.Fragment>
      {isOver && hoverBehavior === 'insert' && dragItem && (
        <UUIDBlock
          ancestors={ancestors}
          idx={0}
          data={dragItem}
          context={context}
          dragDisabled
          dropDisabled
          dragBehavior='move' />
      )}
      <div ref={dropDisabled ? null : drop}>
        <div ref={preview} hidden={isDragging && dragBehavior === 'move'} style={blockStyle} className={focused ? `focus-${frame}` : null} >
          <Row wrap={false} style={{ fontSize: 16, display: 'flex', flexDirection: 'row' }} align='middle' justify='space-between'>
            <Row ref={editing ? null : drag} wrap={false} align='middle' style={{ boxShadow: editing ? 'inset 0px 0px 2px 1px #ffffff' : null, borderColor: 'white', backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, textAlign: 'start', flex: 1, minWidth: 130, maxWidth: 200, cursor: dragDisabled ? "not-allowed" : "grab", zIndex: 101, marginRight: 5, height: 32 }}>
              <Icon style={{ marginLeft: 5 }} component={ICONS[itemType]} />
              <Input style={{ maxWidth: 200, color: 'white', cursor: editing ? 'text' : dragDisabled ? "not-allowed" : "grab" }} bordered={false} disabled={!editing} value={itemType === 'placeholder' ? displayData.pending_node.name : displayData.name} onChange={(e) => onNameChange(e.target.value)} />
            </Row>
            <Row wrap={false} style={{ width: 60, textTransform: 'capitalize', textAlign: 'right' }} align='middle' justify='end'>
              {displayData.editable ? <UnlockOutlined style={{ marginRight: isReal ? 0 : 5 }} /> : <LockOutlined style={{ marginRight: isReal ? 0 : 5 }} />}
              {/* {isArgument && displayData.editable && !editing && <EditOutlined onClick={() => setEditing(true)}/>} */}
              {showMore && (
                <Dropdown overlay={
                  <Menu>
                    {isReal && (
                      <Menu.Item key='show' onClick={({ domEvent }) => { domEvent.stopPropagation(); clearFocusItem(); setFocusItem(itemType, data.uuid) }}>
                        <EyeOutlined />{' '}Show {itemType === 'placeholder' ? 'thing' : itemType}
                      </Menu.Item>
                    )}
                    {onNameChange && (
                      <Menu.Item key='show' onClick={({ domEvent }) => { domEvent.stopPropagation(); setEditing(!editing) }}>
                        <EditOutlined />{editing ? " Done Editing" : ' Edit Name'}
                      </Menu.Item>
                    )}
                    {!inDrawer && data.deleteable &&
                      <Menu.Item key='clear' onClick={onDelete}>
                        <DeleteOutlined />{' '}Clear
                      </Menu.Item>}
                  </Menu>
                }>
                  <Button
                    type='text'
                    style={{ marginLeft: 5 }}
                    icon={<EllipsisOutlined />}
                  />
                </Dropdown>

              )}
              {!isReal && data.deleteable &&
                <Button
                  type='text'
                  style={{ marginLeft: 0 }}
                  onClick={onDelete}
                  icon={<DeleteOutlined />}
                />
              }
            </Row>
          </Row>
        </div>
      </div>
      {!(isDragging && dragBehavior === 'move') && after}
    </React.Fragment>
  );
};
