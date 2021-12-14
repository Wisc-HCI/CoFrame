import React, { useState, useCallback } from "react";
import { Button, Dropdown, Menu } from "antd";
// import { useDrag, useDrop } from 'react-dnd';
import { EllipsisOutlined, DeleteOutlined, EyeOutlined, EditOutlined } from '@ant-design/icons';
import useStore from "../../../stores/Store";
import shallow from 'zustand/shallow';
import blockStyles from "../blockStyles";
import { ReactComponent as LocationIcon } from '../../CustomIcons/Location.svg';
import { ReactComponent as MachineIcon } from '../../CustomIcons/Gear.svg';
import { ReactComponent as ThingIcon } from '../../CustomIcons/Thing.svg';
import { ReactComponent as WaypointIcon } from '../../CustomIcons/Waypoint.svg';
import { ReactComponent as ContainerIcon } from '../../CustomIcons/Container.svg';
import '../highlight.css';
import { Base } from "./Base";

const ICONS = {
  'machine': MachineIcon,
  'location': LocationIcon,
  'thing': ThingIcon,
  'waypoint': WaypointIcon,
  'trajectory': ContainerIcon
}

export const UUIDBlock = ({ data, dragHandle, ancestors, context, dragDisabled, style }) => {

  // refData should always refer to something in the store or context
  const [refData, real] = useStore(useCallback(state => {
    if (state.data[data.ref]) { return [state.data[data.ref], true] }
    else if (context[data.ref]) { return [context[data.ref], false] }
  }, [data, context]))

  const [editing, setEditing] = useState(false);

  const [
    focusItem, setItemProperty, deleteItem,
    setFocusItem, clearFocusItem] = useStore(state => ([
      state.focusItem, state.setItemProperty, state.deleteItem,
      state.setFocusItem, state.clearFocusItem]
    ), shallow);
  const focused = focusItem.uuid === refData.uuid;

  const inDrawer = ancestors[0].uuid === 'drawer';

  const blockStyle = {
    backgroundColor:
      blockStyles[refData.type],
    height: 43,
    width: '100%',
    borderRadius: 3,
    padding: 5,
    position: 'relative',
    zIndex: focused ? 100 : 50,
    opacity: 1,
    ...style
  };

  return (
    <Base
      dragHandle={dragHandle}
      dragDisabled={dragDisabled}
      focused={focused}
      locked={refData.readonly}
      name={refData.name}
      nameEditable={editing}
      onNameChange={(v) => setItemProperty(refData.uuid, 'name', v)}
      type={refData.type}
      extra={real && (
        <Dropdown overlay={
          <Menu>
            {real && (
              <Menu.Item key='show' onClick={({ domEvent }) => { domEvent.stopPropagation(); clearFocusItem(); setFocusItem('data', refData.uuid) }}>
                <EyeOutlined />{' '}<span style={{ textTransform: "capitalize" }}>Show {refData.type}</span>
              </Menu.Item>
            )}
            {real && (
              <Menu.Item key='edit' onClick={({ domEvent }) => { domEvent.stopPropagation(); setEditing(!editing) }}>
                <EditOutlined />{editing ? " Done Editing" : ' Edit Name'}
              </Menu.Item>
            )}
            {!inDrawer && !refData.readonly &&
              <Menu.Item key='delete' onClick={() => deleteItem(data.uuid)}>
                <DeleteOutlined />{' '}Clear
              </Menu.Item>}
          </Menu>
        }>
          <Button
            type='text'
            icon={<EllipsisOutlined />}
          />
        </Dropdown>

      )}
    />
  )
};
