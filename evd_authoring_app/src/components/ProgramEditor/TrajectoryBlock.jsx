import React, { useCallback, useState } from "react";
import useStore from "../../stores/Store";
import blockStyles from "./blockStyles";
import { NodeZone } from "./NodeZone";
import Icon, { UnlockOutlined, LockOutlined, DownOutlined, RightOutlined, EllipsisOutlined } from '@ant-design/icons';
import { Row, Col, Button, InputNumber } from 'antd';
import { ReactComponent as ContainerIcon } from '../CustomIcons/Container.svg'
import './highlight.css';
import { UUIDBlock } from "./UUIDBlock";
import { SortableSeparator } from "./SortableSeparator";
import { useDrag } from "react-dnd";
import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';
import useMeasure from "react-use-measure";

export const TrajectoryBlock = ({ staticData, uuid, ancestors, context, parentData, dragBehavior, dragDisabled, onDelete }) => {
  const [focused, data, start_location, waypoints, end_location] = useStore(useCallback(state => {
    const data = staticData ? staticData : state.data.trajectories[uuid];
    return [
    state.focusItem.uuid === data.uuid,
    data,
    data.start_location_uuid ? state.data.locations[data.start_location_uuid] : null,
    data.waypoint_uuids.map(uuid => state.data.waypoints[uuid]),
    data.end_location_uuid ? state.data.locations[data.end_location_uuid] : null,
  ]}, [staticData, uuid]));

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
    ]);
  const unfocused = focusExists && !focused;

  const inDrawer = ancestors[0].uuid === 'drawer';
  const editingEnabled = !inDrawer && data.editable;

  const [{ isDragging }, drag, preview] = useDrag(() => ({
    type: data.type,
    item: { ...data, parentData, dragBehavior, onDelete },
    options: { dragEffect: dragBehavior },
    canDrag: _ => !dragDisabled,
    collect: monitor => ({
      isDragging: monitor.isDragging()
    })
  }))

  const [settingsExpanded, setSettingsExpanded] = useState(false);

  const [settingsRef, { height }] = useMeasure();
  const settingsStyle = useSpring({ height: height, config: config.stiff});

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

  const startDrop = (dropData) => {
    if (dropData.parentData.uuid === uuid && dropData.dragBehavior === 'move') {
      setItemProperty('trajectory', uuid, dropData.parentData.field, null);
      setItemProperty('trajectory', uuid, 'start_location_uuid', dropData.uuid);
    } else {
      setItemProperty('trajectory', uuid, 'start_location_uuid', dropData.uuid);
    }
  }

  const waypointDrop = (dropData,idx) => {
    if (dropData.parentData.uuid === uuid && dropData.dragBehavior === 'move') {
      const newIdx = dropData.idx <= idx ? idx-1 : idx;
      if (newIdx === dropData.idx) {
        return
      }
      moveTrajectoryWaypoint(dropData.uuid, dropData.parentData.uuid, uuid, dropData.idx, newIdx);
    } else {
      insertTrajectoryWaypoint(dropData.uuid, uuid, idx);
    }
  }

  const endDrop = (dropData) => {
    if (dropData.parentData.uuid === uuid && dropData.dragBehavior === 'move') {
      setItemProperty('trajectory', uuid, dropData.parentData.field, null);
      setItemProperty('trajectory', uuid, 'end_location_uuid', dropData.uuid);
    } else {
      setItemProperty('trajectory', uuid, 'end_location_uuid', dropData.uuid);
    }
  }

  return (
    <div hidden={isDragging && dragBehavior==='move'} ref={preview} style={styles} className={focused ? `focus-${frame}` : null} onClick={(e) => { e.stopPropagation(); unfocused && clearFocusItem() }}>
      <Row style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
        <Col ref={drag} span={17} style={{ backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, textAlign: 'start', cursor:'grab' }}>
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
        <animated.div style={{ overflow: 'hidden', ...settingsStyle }}>
          <div ref={settingsRef}>
            {settingsExpanded && (
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
            </Row>)}
            {settingsExpanded && (
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
            </Row>)}
            </div>
          </animated.div>
      </div>
      <Row align="middle" style={fieldStyle}>
        <Col span="8">Start Location:</Col>
        <Col span="16">
          <NodeZone
            ancestors={trajectoryLocationAncestors}
            context={context}
            onDrop={startDrop}
            emptyMessage='No Start Location'
            dropDisabled={!editingEnabled}
          >
            {start_location && (
              <UUIDBlock
                key={start_location.uuid}
                id={start_location.uuid}
                idx={0}
                dragBehavior='move'
                hoverBehavior='replace'
                ancestors={trajectoryLocationAncestors}
                context={context}
                parentData={{type:'trajectory',uuid,field:'start_location_uuid'}}
                data={{ ...start_location, itemType: 'location', type: `uuid-location` }}
                onDelete={(_) => setItemProperty('trajectory', uuid, 'start_location_uuid', null)}
                onDrop={startDrop}
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
            onDrop={(dropData)=>waypointDrop(dropData,0)}
            emptyMessage='No Waypoints'
            dropDisabled={!editingEnabled}
          >
            {waypoints.map((waypoint, idx) => (
              <div key={idx}>
                {idx === 0 && (
                  <SortableSeparator
                    ancestors={trajectoryWaypointAncestors}
                    height={30}
                    spacing={idx===0 ? 0 : 5}
                    context={context}
                    onDrop={(dropData)=>waypointDrop(dropData,0)}
                    dropDisabled={!editingEnabled}
                  />
                )}
                <UUIDBlock
                  key={waypoint.uuid}
                  id={waypoint.uuid}
                  idx={idx}
                  dragBehavior='move'
                  ancestors={trajectoryWaypointAncestors}
                  context={context}
                  parentData={{type:'trajectory',uuid,field:'waypoint_uuids'}}
                  data={{ ...waypoint, itemType: 'waypoint', type: `uuid-waypoint` }}
                  onDelete={() => deleteTrajectoryWaypoint(uuid, idx)}
                  // nameEditable={}
                  // onNameChange={(newName)=>{}}
                  dragDisabled={!editingEnabled}
                  dropDisabled={true}
                  after={
                    <SortableSeparator
                    ancestors={trajectoryWaypointAncestors}
                    context={context}
                    height={30}
                    end={idx === waypoints.length-1}
                    spacing={idx === waypoints.length-1 ? 0 : 5}
                    onDrop={(dropData)=>waypointDrop(dropData,idx+1)}
                    dropDisabled={!editingEnabled}
                  />
                  }
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
            ancestors={trajectoryLocationAncestors}
            onDrop={endDrop}
            emptyMessage='No End Location'
            dropDisabled={!editingEnabled}
          >
            {end_location && (
              <UUIDBlock
                key={end_location.uuid}
                id={end_location.uuid}
                idx={0}
                hoverBehavior='replace'
                dragBehavior='move'
                parentData={{type:'trajectory',uuid,field:'end_location_uuid'}}
                ancestors={trajectoryLocationAncestors}
                context={context}
                data={{ ...end_location, itemType: 'location', type: `uuid-location` }}
                onDelete={(_) => setItemProperty('trajectory', uuid, 'end_location_uuid', null)}
                onDrop={endDrop}
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
};