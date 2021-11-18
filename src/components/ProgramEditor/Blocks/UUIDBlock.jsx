import React, { useState, useCallback } from "react";
import { Button, Dropdown, Input, Menu, Row } from "antd";
// import { useDrag, useDrop } from 'react-dnd';
import Icon, { UnlockOutlined, LockOutlined, EllipsisOutlined, DeleteOutlined, EyeOutlined, EditOutlined } from '@ant-design/icons';
import useStore from "../../../stores/Store";
import shallow from 'zustand/shallow';
import blockStyles from "../blockStyles";
import { ReactComponent as LocationIcon } from '../../CustomIcons/Location.svg';
import { ReactComponent as MachineIcon } from '../../CustomIcons/Gear.svg';
import { ReactComponent as ThingIcon } from '../../CustomIcons/Thing.svg';
import { ReactComponent as WaypointIcon } from '../../CustomIcons/Waypoint.svg';
import { ReactComponent as ContainerIcon } from '../../CustomIcons/Container.svg';
import '../highlight.css';

const ICONS = {
  'machine': MachineIcon,
  'location': LocationIcon,
  'thing': ThingIcon,
  'waypoint': WaypointIcon,
  'trajectory': ContainerIcon
}

export const UUIDBlock = ({ data, ancestors, context, dragDisabled,listeners,attributes }) => {

  // refData should always refer to something in the store or context
  const [refData, real] = useStore(useCallback(state => {
    if (state.data[data.ref]) { return [state.data[data.ref], true] }
    else if (context[data.ref]) { return [context[data.ref], false] }
  }, [data, context]))

  const [editing, setEditing] = useState(false);

  const [
    frame, focusItem, setItemProperty, deleteItem,
    setFocusItem, clearFocusItem] = useStore(state => ([
      state.frame, state.focusItem, state.setItemProperty, state.deleteItem,
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
    zIndex: focused ? 100 : 1,
    opacity: 1
  };

  const handleStuff = listeners && attributes ? {
    ...listeners,
    ...attributes
  } : {}

  return (
    <div style={blockStyle} className={focused ? `focus-${frame}` : null} >
      <Row wrap={false} style={{ fontSize: 16, display: 'flex', flexDirection: 'row' }} align='middle' justify='space-between'>
        <Row {...handleStuff} wrap={false} align='middle' style={{ boxShadow: editing ? 'inset 0px 0px 2px 1px #ffffff' : null, borderColor: 'white', backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, textAlign: 'start', flex: 1, minWidth: 130, maxWidth: 200, cursor: dragDisabled ? "not-allowed" : "grab", zIndex: 101, marginRight: 5, height: 32 }}>
          <Icon style={{ marginLeft: 5 }} component={ICONS[refData.type]} />
          <Input style={{ maxWidth: 200, color: 'white', cursor: editing ? 'text' : "grab" }} bordered={false} disabled={!editing} value={refData.name} onChange={(e) => setItemProperty(refData.uuid, 'name', e.target.value)} />
        </Row>
        <Row wrap={false} style={{ width: 60, textTransform: 'capitalize', textAlign: 'right' }} align='middle' justify='end'>
          {!refData.readonly ? <UnlockOutlined style={{ marginRight: real ? 0 : 5 }} /> : <LockOutlined style={{ marginRight: 5 }} />}
          {/* {isArgument && refData.editable && !editing && <EditOutlined onClick={() => setEditing(true)}/>} */}
          {real && (
            <Dropdown overlay={
              <Menu>
                {real && (
                  <Menu.Item key='show' onClick={({ domEvent }) => { domEvent.stopPropagation(); clearFocusItem(); setFocusItem('data', refData.uuid) }}>
                    <EyeOutlined />{' '}<span style={{ textTransform: "capitalize" }}>Show {refData.type}</span>
                  </Menu.Item>
                )}
                {real && (
                  <Menu.Item key='show' onClick={({ domEvent }) => { domEvent.stopPropagation(); setEditing(!editing) }}>
                    <EditOutlined />{editing ? " Done Editing" : ' Edit Name'}
                  </Menu.Item>
                )}
                {!inDrawer && !refData.readonly &&
                  <Menu.Item key='clear' onClick={() => deleteItem(data.uuid)}>
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
        </Row>
      </Row>
    </div>
  );
};
