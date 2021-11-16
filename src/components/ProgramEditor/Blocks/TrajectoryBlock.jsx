import React, { useCallback } from "react";
import useStore from "../../../stores/Store";
import shallow from 'zustand/shallow';
import blockStyles from "../blockStyles";
import { NodeZone } from "../NodeZone";
import { NodeList } from "../NodeList";
import Icon, { UnlockOutlined, LockOutlined, EllipsisOutlined, EyeOutlined } from '@ant-design/icons';
import { Row, Col, Button, Dropdown, Menu } from 'antd';
import { ReactComponent as ContainerIcon } from '../../CustomIcons/Container.svg'
import '../highlight.css';

export const TrajectoryBlock = ({ data, ancestors, context, dragDisabled, dropDisabled, listeners,attributes }) => {

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
    zIndex: focused ? 100 : 1
  };

  const handleStuff = listeners && attributes ? {
    ...listeners,
    ...attributes
  } : {}

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
    <div style={styles} className={focused ? `focus-${frame}` : null} onClick={(e) => { e.stopPropagation(); unfocused && clearFocusItem() }}>
      <Row style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
        <Col {...handleStuff} span={17} style={{ backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, textAlign: 'start', cursor: dragDisabled ? 'not-allowed' : 'grab' }}>
          <Icon style={{ marginLeft: 4 }} component={ContainerIcon} />{' '}{data.name}
        </Col>
        <Col span={6} offset={1} style={{ textAlign: 'end' }}>
          {editingEnabled ? <UnlockOutlined /> : <LockOutlined />}
          <Dropdown overlay={
            <Menu>
              <Menu.Item key='show' onClick={({ domEvent }) => { domEvent.stopPropagation(); clearFocusItem(); setFocusItem('trajectory', data.uuid) }}>
                <EyeOutlined />{' '}Show Trajectory
              </Menu.Item>
            </Menu>
          }>
            <Button
              type='text'
              style={{ marginLeft: 5 }}
              icon={<EllipsisOutlined />}
            />
          </Dropdown>
        </Col>
      </Row>
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
    </div>
  );
};