import React, {useState} from "react";
import { Button, Dropdown, Input, Menu, Row, Tooltip } from "antd";
import { useDrag, useDrop } from 'react-dnd';
import Icon, { UnlockOutlined, LockOutlined, EllipsisOutlined, DeleteOutlined, EyeOutlined, EditOutlined } from '@ant-design/icons';
import useStore from "../../stores/Store";
import blockStyles from "./blockStyles";
import { ReactComponent as LocationIcon } from '../CustomIcons/Location.svg';
import { ReactComponent as MachineIcon } from '../CustomIcons/Gear.svg';
import { ReactComponent as ThingIcon } from '../CustomIcons/Thing.svg';
import { ReactComponent as WaypointIcon } from '../CustomIcons/Waypoint.svg';
import './highlight.css';

const ICONS = {
  'machine': MachineIcon,
  'location': LocationIcon,
  'placeholder': ThingIcon,
  'waypoint': WaypointIcon
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
  const [editing, setEditing] = useState(data.name === '');
  let editInputValue;
  let saveEditInputRef;

  const [{ isDragging }, drag, preview] = useDrag(() => ({
    type: data.type,
    item: { ...data, parentData, dragBehavior, idx, onDelete },
    options: { dragEffect: dragBehavior },
    canDrag: _ => !dragDisabled,
    collect: monitor => ({
      isDragging: monitor.isDragging()
    })
  }))

  // We only care about the second value returned from useDrop (hence the [1] at the end)
  const [{ isOver, dragItem }, drop] = useDrop({
    accept: ancestors[0].accepts,
    drop: (otherItem, _) => {
      onDrop(otherItem)
    },
    canDrop: (otherItem, _) => {
      if (dropDisabled || !data.editable) {
        return false
      } else if (!ancestors[0].accepts.some(type => type === otherItem.type)) {
        return false
      } else if (!ancestors.some(ancestor => ancestor.uuid === otherItem.parentData.uuid) && otherItem.parentData.type !== 'drawer') {
        return false
      }
      return true;
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
    ));
  const focused = focusItem.uuid === uuid;

  const inDrawer = ancestors[0].uuid === 'drawer';
  const showMore = context[uuid]?.real;

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

  // Unformatted display name information
  const displayData = isOver && hoverBehavior === 'replace' && dragItem ? dragItem : data;
  const tempDisplayName = (typeof displayData.name !== 'undefined') ? (itemType === 'placeholder' ? displayData.pending_node.name : displayData.name) : "";

  // Length of a long tag and boolean for if the name meets the criteria
  const longLength = inDrawer ? 16 : 24;
  const isLongTag = tempDisplayName.length > longLength;

  // Formatted display name
  const displayName = isLongTag ? `${tempDisplayName.slice(0, longLength-3)}...` : tempDisplayName;

  // Icon adjustment and argument detection
  const isArgument = data.uuid.includes('skill-arg');
  const iconTrayWidth = isArgument ? 90 : 70;

  // Ends edit mode and calls function for onNameChange
  const onConfirmName = () => {
    setEditing(false);
    onNameChange(editInputValue);
  }

  // Updates local variable as the Input field changes
  const handleEditChange = e => {
    editInputValue = e.target.value;
  }

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
        <div ref={preview} hidden={isDragging&&dragBehavior==='move'} style={blockStyle} className={focused ? `focus-${frame}` : null} >
          <Row wrap={false} style={{ fontSize: 16, display: 'flex', flexDirection: 'row' }} align='middle' justify='space-between'>
            <span ref={drag} style={{ backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, textAlign: 'start', flex: 1, minWidth: 130, cursor: "grab",zIndex:101, marginRight:5 }}>
            <Icon component={ICONS[itemType]} />{' '}{!isArgument ? 
              (isLongTag ? <Tooltip title={tempDisplayName}>{displayName}</Tooltip> : displayName) : (
                editing ? <Input ref={saveEditInputRef}
                                      style={{ padding: 4, flex: 1, width: "auto" }}
                                      value={editInputValue} 
                                      onChange={handleEditChange} 
                                      onBlur={onConfirmName} 
                                      onPressEnter={onConfirmName}/> : (
                                        isLongTag ? <Tooltip title={tempDisplayName}>{displayName}</Tooltip> : displayName
                                        ))}
            </span>
            <span style={{ textAlign: 'end', width: iconTrayWidth, textTransform: 'capitalize' }}>
              {displayData.editable ? <UnlockOutlined style={{marginRight: showMore? 0 : 5}}/> : <LockOutlined style={{marginRight: showMore? 0 : 5}}/>}
              {isArgument && displayData.editable && !editing && <EditOutlined onClick={() => setEditing(true)}/>}
              {showMore && (
                <Dropdown overlay={
                  <Menu>
                    <Menu.Item key='show' onClick={({ domEvent }) => { domEvent.stopPropagation(); clearFocusItem(); setFocusItem(itemType, data.uuid) }}>
                      <EyeOutlined />{' '}Show {itemType === 'placeholder' ? 'thing' : itemType}
                    </Menu.Item>
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
              {!showMore && data.deleteable &&
                  <Button
                  type='text'
                  style={{ marginLeft: 0 }}
                  onClick={onDelete}
                  icon={<DeleteOutlined />}
                />
              }
            </span>
          </Row>
        </div>
      </div>
      {!(isDragging && dragBehavior==='move') && after}
    </React.Fragment>
  );
};
