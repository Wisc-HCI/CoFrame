import React, { forwardRef } from "react";
import { Button, Dropdown, Menu } from "antd";
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

export const UUIDBlock = forwardRef(({ data, preview, style, ancestors, context, onDelete }, ref) => {

  // props constains data,
  // which contains fields 'itemType' and 'uuid'
  // console.log(data)
  const { itemType, uuid } = data;

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
    // minWidth: 150,
    borderRadius: 3,
    margin: 4,
    padding: 5,
    position: 'relative',
    zIndex: focused ? 100 : 1
  };
  // if (itemType === 'placeholder') { console.log(data) }
  // const name = useStore(useCallback(state=>itemType==='placeholder'?state.data.placeholders[uuid].pending_node.name:state.data[typeToKey(itemType)][uuid].name,[itemType,uuid]));
  
  return (
    <div ref={preview} style={{ ...style, ...blockStyle }} className={focused ? `focus-${frame}` : null} >
      <span style={{ fontSize: 16, display: 'flex', flexDirection: 'row' }} align='middle' justify='space-between'>
        <span ref={ref} style={{backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, textAlign: 'start', flex: 1, minWidth: 130, cursor: "grab" }}>
          <Icon style={{ marginLeft: 4 }} component={ICONS[itemType]} />{' '}{itemType === 'placeholder' ? data.pending_node.name : data.name}
        </span>
        <span style={{ textAlign: 'end', width: 60 , textTransform:'capitalize' }}>
          {data.editable ? <UnlockOutlined /> : <LockOutlined />}
          {showMore ? (
            <Dropdown overlay={
              <Menu>
                <Menu.Item onClick={({domEvent}) => { domEvent.stopPropagation(); clearFocusItem(); setFocusItem(itemType, data.uuid) }}>
                  <EyeOutlined/>{' '}Show {itemType === 'placeholder' ? 'thing' : itemType}
                </Menu.Item>
                {!inDrawer && 
                <Menu.Item onClick={onDelete}>
                  <DeleteOutlined/>{' '}Clear
                </Menu.Item>}
              </Menu>
            }>
              <Button
                type='text'
                style={{ marginLeft: 2 }}
                icon={<EllipsisOutlined />}
              />
            </Dropdown>
            
          ):(
            <Button
              type='text'
              style={{ marginLeft: 2 }}
              onClick={onDelete}
              icon={<DeleteOutlined/>}
          />
          )}
        </span>
      </span>
    </div>
  );
});
