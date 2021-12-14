import React, { useCallback } from "react";
import useStore from "../../../stores/Store";
import shallow from 'zustand/shallow';
import blockStyles from "../blockStyles";
import { NodeZone } from "../NodeZone";
import { NodeList } from "../NodeList";
import { EllipsisOutlined, EyeOutlined } from '@ant-design/icons';
import { Row, Col, Button, Dropdown, Menu } from 'antd';
import { Base } from "./Base";
import '../highlight.css';

export const TrajectoryBlock = ({ data, dragHandle, ancestors, context, dragDisabled, dropDisabled, style }) => {

  const focused = useStore(useCallback(state => state.focusItem.uuid === data.uuid, [data]), shallow);

  const [
    frame, clearFocusItem, setItemProperty, setFocusItem,
    moveTrajectoryWaypoint, insertTrajectoryWaypoint,
    deleteTrajectoryWaypoint, focusExists] = useStore(state => [
      state.frame,
      state.clearFocusItem,
      state.setItemProperty,
      state.setFocusItem,
      state.moveTrajectoryWaypoint,
      state.insertTrajectoryWaypoint,
      state.deleteTrajectoryWaypoint,
      state.focusItem.type !== null
    ], shallow);
  const unfocused = focusExists && !focused;

  const inDrawer = ancestors[0].uuid === 'drawer';
  const editingEnabled = !inDrawer && !data.readonly;

  const styles = {
    backgroundColor:
      blockStyles[data.type],
    minHeight: 30,
    minWidth: 250,
    borderRadius: 3,
    margin: 0,
    padding: 5,
    position: 'relative',
    zIndex: focused ? 100 : 1,
    ...style
  };

  const trajectoryWaypointAncestors = [
    { uuid: data.uuid, accepts: ['uuid-waypoint'] },
    ...ancestors
  ];

  const trajectoryLocationAncestors = [
    { uuid: data.uuid, accepts: ['uuid-location'] },
    ...ancestors
  ];

  const fieldStyle = {
    borderRadius: 4,
    margin: '1pt 1pt 4pt 1pt',
    padding: 5,
    backgroundColor: "rgba(0,0,0,0.1)"
  }

  return (
    <Base
      dragHandle={dragHandle}
      dragDisabled={dragDisabled}
      focused={focused}
      locked={data.readonly}
      name={data.name}
      nameEditable={false}
      type='trajectory'
      extra={<Dropdown overlay={
          <Menu>
            <Menu.Item key='show' onClick={({ domEvent }) => { domEvent.stopPropagation(); clearFocusItem(); setFocusItem('data', data.uuid) }}>
              <EyeOutlined />{' '}Show Trajectory
            </Menu.Item>
          </Menu>
        }>
          <Button
            type='text'
            style={{ marginLeft: 5 }}
            icon={<EllipsisOutlined />}
          />
        </Dropdown>}
    >
      <Row align="middle" style={fieldStyle}>
        <Col flex={2} style={{ paddingRight: 5 }}>Start Location:</Col>
        <Col flex={3}>
          <NodeZone
            parentId={data.uuid}
            field='startLocation'
            ancestors={trajectoryLocationAncestors}
            context={context}
            uuid={data.startLocation}
            dragDisabled={!editingEnabled}
            dropDisabled={dropDisabled}
          />
        </Col>
      </Row>
      <Row align="middle" style={fieldStyle}>
        <Col span="8">Waypoints:</Col>
        <Col span="16">
          <NodeList ancestors={trajectoryWaypointAncestors} field='waypoints' uuids={data.waypoints} context={context} dragDisabled={!editingEnabled} dropDisabled={dropDisabled}/>
        </Col>
      </Row>
      <Row align="middle" style={fieldStyle}>
        <Col flex={2} style={{ paddingRight: 5 }}>End Location:</Col>
        <Col flex={3}>
          <NodeZone
            parentId={data.uuid}
            field='endLocation'
            ancestors={trajectoryLocationAncestors}
            context={context}
            uuid={data.endLocation}
            dragDisabled={!editingEnabled}
            dropDisabled={dropDisabled}
          />
        </Col>
      </Row>
    </Base>
  )
};