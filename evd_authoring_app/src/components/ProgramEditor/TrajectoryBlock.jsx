import React, { forwardRef, useCallback, useState } from "react";
import useStore from "../../stores/Store";
import blockStyles from "./blockStyles";
import { NodeZone } from "./NodeZone";
import Icon, { UnlockOutlined, LockOutlined, DownOutlined, RightOutlined, EllipsisOutlined } from '@ant-design/icons';
import { Row, Col, Button, InputNumber } from 'antd';
import { ReactComponent as ContainerIcon } from '../CustomIcons/Container.svg'
import './highlight.css';
import { UUIDBlock } from "./UUIDBlock";
import { SortableSeparator } from "./SortableSeparator";

export const TrajectoryBlock = forwardRef(({ data, ancestors, preview, style, context }, ref) => {
  const { uuid, start_location_uuid, waypoint_uuids, end_location_uuid } = data;
  const [focused, start_location, waypoints, end_location] = useStore(useCallback(state => [
    state.focusItem.uuid === uuid,
    start_location_uuid ? state.data.locations[start_location_uuid] : null,
    waypoint_uuids.map(uuid => state.data.waypoints[uuid]),
    end_location_uuid ? state.data.locations[end_location_uuid] : null,
  ], [uuid, start_location_uuid, waypoint_uuids, end_location_uuid]));
  const [
    frame, clearFocusItem, setItemProperty, setFocusItem,
    moveTrajectoryWaypoint, deleteTrajectoryWaypoint, focusExists] = useStore(state => [
      state.frame,
      state.clearFocusItem,
      state.setItemProperty,
      state.setFocusItem,
      state.moveTrajectoryWaypoint,
      state.deleteTrajectoryWaypoint,
      state.focusItem.type !== null
    ]);
  const unfocused = focusExists && !focused;

  const inDrawer = ancestors[0].uuid === 'drawer';
  const editingEnabled = !inDrawer && data.editable;

  const [settingsExpanded, setSettingsExpanded] = useState(false);

  const styles = {
    backgroundColor:
      blockStyles[data.type],
    minHeight: 30,
    minWidth: 250,
    borderRadius: 3,
    margin: 4,
    padding: 5,
    position: 'relative',
    zIndex: focused ? 100 : 1
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

  const selectionStyle = {
    backgroundColor: 'rgba(255,255,255,0.2)',
    borderRadius: 2,
    margin: '0pt 8pt 12pt 0pt',
    padding: '6pt 15pt',
    fontSize: 14,
    height: 36
  }

  return (
    <div ref={preview} style={{ ...style, ...styles }} className={focused ? `focus-${frame}` : null} onClick={(e) => { e.stopPropagation(); unfocused && clearFocusItem() }}>
      <Row style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
        <Col ref={ref} span={17} style={{ backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, textAlign: 'start' }}>
          <Icon style={{ marginLeft: 4 }} component={ContainerIcon} />{' '}{data.name}
        </Col>
        <Col span={6} offset={1} style={{ textAlign: 'end' }}>
          {editingEnabled ? <UnlockOutlined /> : <LockOutlined />}
          <Button
            type='text'
            style={{ marginLeft: 2 }}
            onClick={(e) => { e.stopPropagation(); clearFocusItem(); setFocusItem('trajectory', data.uuid) }}
            icon={<EllipsisOutlined />}
          />
        </Col>
      </Row>
      <div style={fieldStyle}>
        <Row align="middle" style={{ marginBottom: 5 }}>

          <Col span="20">Settings:</Col>
          <Col span="4">
            <Button
              disabled={inDrawer}
              onClick={() => setSettingsExpanded(!settingsExpanded)}
              type='text'
              icon={settingsExpanded ? (
                <DownOutlined />
              ) : (
                <RightOutlined />
              )}
            />
          </Col>
        </Row>
        {settingsExpanded && (
          <>
            <Row align="middle" style={fieldStyle}>
              <Col span="8">Motion Type:</Col>
              <Col span="16" style={{ textAlign: 'right' }}>
                {data.move_type === 'ee_ik' ? (
                  <span style={selectionStyle}>IK</span>
                ) : (
                  <Button type='text' disabled={!editingEnabled} onClick={() => setItemProperty('trajectory', uuid, 'move_type', 'ee_ik')}>IK</Button>
                )}
                {data.move_type === 'joint' ? (
                  <span style={selectionStyle}>Joint</span>
                ) : (
                  <Button type='text' disabled={!editingEnabled} onClick={() => setItemProperty('trajectory', uuid, 'move_type', 'joint')}>IK</Button>
                )}
              </Col>
            </Row>
            <Row align="middle" style={fieldStyle}>
              <Col span="8">Speed:</Col>
              <Col span="16" style={{ textAlign: 'right' }}>
                <InputNumber
                  min={0.01}
                  max={5}
                  size='small'
                  defaultValue={data.velocity}
                  disabled={!editingEnabled}
                  onChange={(v) => setItemProperty('trajectory', uuid, 'move_type', v)}
                  bordered={false}
                  style={{ backgroundColor: 'rgba(255,255,255,0.2)' }} />
              </Col>
            </Row>
          </>
        )}
      </div>
      <Row align="middle" style={fieldStyle}>
        <Col span="8">Start Location:</Col>
        <Col span="16">
          <NodeZone
            style={{paddingTop:4,paddingBottom:4}}
            ancestors={trajectoryLocationAncestors}
            context={context}
            onDrop={(dropData) => setItemProperty('trajectory', uuid, 'start_location_uuid', dropData.uuid)}
            emptyMessage='No Start Location'
            dropDisabled={!editingEnabled}
          >
            {start_location && (
              <UUIDBlock
                key={start_location.uuid}
                id={start_location.uuid}
                idx={0}
                hoverBehavior='replace'
                ancestors={trajectoryLocationAncestors}
                context={context}
                parentData={{type:'trajectory',uuid}}
                data={{ ...start_location, itemType: 'location', type: `uuid-location` }}
                onDelete={(_) => setItemProperty('trajectory', uuid, 'start_location_uuid', null)}
                onDrop={(dropData) => setItemProperty('trajectory', uuid, 'start_location_uuid', dropData.uuid)}
                dragDisabled={!editingEnabled}
                dropDisabled={!editingEnabled}
              />
            )
            }
          </NodeZone>
        </Col>
      </Row>
      <Row align="middle" style={fieldStyle}>
        <Col span="8">Waypoints:</Col>
        <Col span="16">
          <NodeZone
            ancestors={trajectoryWaypointAncestors}
            context={context}
            onDrop={(dropData) => moveTrajectoryWaypoint(dropData, uuid, 0)}
            emptyMessage='No Waypoints'
            dropDisabled={!editingEnabled}
          >
            {waypoints.map((waypoint, idx) => (
              <div key={idx}>
                {idx === 0 && (
                  <SortableSeparator
                    ancestors={trajectoryWaypointAncestors}
                    context={context}
                    onDrop={(dropData) => moveTrajectoryWaypoint(dropData, uuid, 0)}
                    dropDisabled={!editingEnabled}
                  />
                )}
                <UUIDBlock
                  key={waypoint.uuid}
                  id={waypoint.uuid}
                  idx={0}
                  hoverBehavior='insert'
                  ancestors={trajectoryWaypointAncestors}
                  context={context}
                  parentData={{type:'trajectory',uuid}}
                  data={{ ...waypoint, itemType: 'waypoint', type: `uuid-waypoint` }}
                  onDelete={() => deleteTrajectoryWaypoint(uuid, idx)}
                  dragDisabled={!editingEnabled}
                  dropDisabled={true}
                />
                <SortableSeparator
                    ancestors={trajectoryWaypointAncestors}
                    context={context}
                    onDrop={(dropData) => moveTrajectoryWaypoint(dropData, uuid, idx+1)}
                    dropDisabled={!editingEnabled}
                />
              </div>

            ))}
          </NodeZone>
        </Col>
      </Row>
      <Row align="middle" style={fieldStyle}>
        <Col span="8">End Location:</Col>
        <Col span="16">
          <NodeZone
            context={context}
            style={{paddingTop:4,paddingBottom:4}}
            ancestors={trajectoryLocationAncestors}
            onDrop={(dropData) => setItemProperty('trajectory', uuid, 'end_location_uuid', dropData.uuid)}
            emptyMessage='No End Location'
            dropDisabled={!editingEnabled}
          >
            {end_location && (
              <UUIDBlock
                key={end_location.uuid}
                id={end_location.uuid}
                idx={0}
                hoverBehavior='replace'
                parentData={{type:'trajectory',uuid}}
                ancestors={trajectoryLocationAncestors}
                context={context}
                data={{ ...end_location, itemType: 'location', type: `uuid-location` }}
                onDelete={(_) => setItemProperty('trajectory', uuid, 'end_location_uuid', null)}
                onDrop={(dropData) => setItemProperty('trajectory', uuid, 'end_location_uuid', dropData.uuid)}
                dragDisabled={!editingEnabled}
                dropDisabled={!editingEnabled}
              />
            )
            }
          </NodeZone>
        </Col>
      </Row>
    </div>
  );
});