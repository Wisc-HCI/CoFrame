import React from "react";
import { Button, Dropdown, Menu } from "antd";
import { useDrag, useDrop } from 'react-dnd';
import Icon, { UnlockOutlined, LockOutlined, EllipsisOutlined, DeleteOutlined, EyeOutlined } from '@ant-design/icons';
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
  ancestors, 
  context, 
  onDelete, 
  onDrop, 
  dragDisabled, 
  dropDisabled, 
  hoverBehavior, 
  dragBehavior, 
  parentData }) => {

  // props constains data,
  // which contains fields 'itemType' and 'uuid'
  // console.log(data)
  // hoverBehavior is either 'replace' or 'insert'
  const { itemType, uuid } = data;

  // const ref = useRef();

  const [{ opacity }, drag, preview] = useDrag(() => ({
    type: data.type,
    item: { ...data, parentData },
    options: { dragEffect: dragBehavior },
    // dragEffect: 'copy',
    collect: monitor => ({
      opacity: monitor.isDragging() ? 0.4 : 1
    })
  }))

  // We only care about the second value returned from useDrop (hence the [1] at the end)
  const [{ isOver, dragItem }, drop] = useDrop({
    accept: ancestors[0].accepts,
    drop: (otherItem, _) => {
      onDrop(otherItem)
    },
    canDrop: (otherItem, _) => {
      if (!otherItem) {
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
    minHeight: 30,
    width: 'calc(100% - 5pt)',
    borderRadius: 3,
    marginLeft: 4,
    marginRight: 4,
    padding: 5,
    position: 'relative',
    zIndex: focused ? 100 : 1,
    opacity: isOver && hoverBehavior === 'replace' && dragItem ? 0.5 : 1
  };

  const displayData = isOver && hoverBehavior === 'replace' && dragItem ? dragItem : data

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
        <div ref={preview} style={{ opacity, ...blockStyle }} className={focused ? `focus-${frame}` : null} >
          <span style={{ fontSize: 16, display: 'flex', flexDirection: 'row' }} align='middle' justify='space-between'>
            <span ref={dragDisabled ? null : drag} style={{ backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, textAlign: 'start', flex: 1, minWidth: 130, cursor: "grab" }}>
              <Icon style={{ marginLeft: 4 }} component={ICONS[itemType]} />{' '}{itemType === 'placeholder' ? displayData.pending_node.name : displayData.name}
            </span>
            <span style={{ textAlign: 'end', width: 60, textTransform: 'capitalize' }}>
              {displayData.editable ? <UnlockOutlined /> : <LockOutlined />}
              {showMore ? (
                <Dropdown overlay={
                  <Menu>
                    <Menu.Item key='show' onClick={({ domEvent }) => { domEvent.stopPropagation(); clearFocusItem(); setFocusItem(itemType, data.uuid) }}>
                      <EyeOutlined />{' '}Show {itemType === 'placeholder' ? 'thing' : itemType}
                    </Menu.Item>
                    {!inDrawer &&
                      <Menu.Item key='clear' onClick={onDelete}>
                        <DeleteOutlined />{' '}Clear
                      </Menu.Item>}
                  </Menu>
                }>
                  <Button
                    type='text'
                    style={{ marginLeft: 2 }}
                    icon={<EllipsisOutlined />}
                  />
                </Dropdown>

              ) : (
                <Button
                  type='text'
                  style={{ marginLeft: 2 }}
                  onClick={onDelete}
                  icon={<DeleteOutlined />}
                />
              )}
            </span>
          </span>
        </div>
      </div>
    </React.Fragment>
  );
};
