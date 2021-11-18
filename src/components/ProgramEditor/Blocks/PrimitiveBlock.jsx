import React, { useCallback, useState } from "react";
import { NodeZone } from "../NodeZone";
// import { ItemSortable } from "./Wrappers";
import { InputNumber, Row, Col, Button } from "antd";
import useStore from "../../../stores/Store";
import shallow from 'zustand/shallow';
import blockStyles from "../blockStyles";
import { ReactComponent as PrimitiveIcon } from '../../CustomIcons/Primitive.svg';
import Icon, { UnlockOutlined, LockOutlined, RightOutlined, EyeOutlined } from '@ant-design/icons';
import '../highlight.css';
import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';

const primitivesWithSettings = [
  'gripper',
  'delay',
  'move-trajectory'
]

const selectionStyle = {
  backgroundColor: 'rgba(255,255,255,0.2)',
  borderRadius: 2,
  margin: '0pt 8pt 12pt 0pt',
  padding: '6pt 15pt',
  fontSize: 14,
  height: 36
}

export const PrimitiveBlock = ({ data, ancestors, context, dragDisabled, dropDisabled,listeners={} }) => {

  const [focused, executable] = useStore(useCallback(state => {
    const executable = state.executablePrimitives[data.uuid] ? true : false;
    return [
      state.focusItem.uuid === data.uuid,
      executable
    ]
  }, [data]), shallow);

  const [frame, setFocusItem, clearFocusItem, focusExists,
    setPrimitiveParameter] = useStore(
      (state) => [
        state.frame, state.setFocusItem,
        state.clearFocusItem, state.focusItem.type !== null,
      ], shallow)

  const unfocused = focusExists && !focused;

  const inDrawer = ancestors[0].uuid === 'drawer';
  const editingEnabled = !inDrawer && data.editable;

  const parameterAncestors = {
    machine: [
      { uuid: data.uuid, accepts: ['uuid-machine'] },
      ...ancestors
    ],
    location: [
      { uuid: data.uuid, accepts: ['uuid-location'] },
      ...ancestors
    ],
    trajectory: [
      { uuid: data.uuid, accepts: ['uuid-trajectory', 'trajectory'] },
      ...ancestors
    ],
    thing: [
      { uuid: data.uuid, accepts: ['uuid-placeholder'] },
      ...ancestors
    ]
  }

  // const [settingsRef, { height }] = useMeasure();
  const [settingsExpanded, setSettingsExpanded] = useState(false);
  const settingsStyle = useSpring({
    opacity: inDrawer || !settingsExpanded ? 0 : 1,
    scaleY: inDrawer || !settingsExpanded ? 0 : 1,
    config: config.stiff
  });
  const carrotStyle = useSpring({
    rotate: inDrawer || !settingsExpanded ? '0deg' : '90deg',
    config: config.wobbly
  });



  const styles = {
    backgroundColor:
      blockStyles[data.type],
    minHeight: 30,
    minWidth: 250,
    borderRadius: 3,
    fontSize: 14,
    padding: 5,
    position: 'relative',
    margin: 0,
    zIndex: focused ? 100 : 1
  };

  const fieldStyle = {
    borderRadius: 4,
    margin: '1pt 1pt 4pt 1pt',
    padding: 5,
    backgroundColor: "rgba(0,0,0,0.1)"
  }

  return (

    <div style={styles} className={focused ? `focus-${frame}` : null} onClick={(e) => { e.stopPropagation(); unfocused && clearFocusItem() }}>
      <Row {...listeners} wrap={false} style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
        <Row wrap={false} align='middle' span={17} style={{ textAlign: 'left', backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, minWidth: 200, maxWidth: 230, cursor: dragDisabled ? 'not-allowed' : 'grab', zIndex: 101 }}>
          <Icon style={{ marginLeft: 4, marginRight: 4 }} component={PrimitiveIcon} />
          {data.name}
        </Row>
        <Row wrap={false} align='middle' style={{ textAlign: 'end' }}>
          {executable && <Button type='text' icon={<EyeOutlined />} onClick={(e) => { e.stopPropagation(); setFocusItem('data', data.uuid) }} />}
          {editingEnabled ? <UnlockOutlined style={{ marginRight: 5, marginLeft: 5 }} /> : <LockOutlined style={{ marginRight: 5, marginLeft: 5 }} />}
          {/* <Button
                type='text'
                style={{marginLeft:2}}
                onClick={(e) => {e.stopPropagation();setFocusItem('program', uuid)}}
                icon={<EllipsisOutlined />}
            /> */}
        </Row>
      </Row>
      {primitivesWithSettings.some(primitive => primitive === data.type) &&
        <div style={fieldStyle}>
          <Row align="middle" style={{ marginBottom: 5 }}>
            <Col span="20">Settings:</Col>
            <Col span="4">
              <Button
                disabled={inDrawer}
                onClick={() => setSettingsExpanded(!settingsExpanded)}
                type='text'
                icon={<animated.div style={carrotStyle}><RightOutlined /></animated.div>}
              />
            </Col>
          </Row>
          <animated.div style={{ overflow: 'hidden', ...settingsStyle }}>
            {settingsExpanded && data.type === 'gripper' && (
              <>
                <Row align="middle" style={fieldStyle}>
                  <Col flex={2}>Position:</Col>
                  <Col flex={3} style={{ textAlign: 'right' }}>
                    <InputNumber
                      min={0}
                      max={85}
                      size='small'
                      suffix='mm'
                      defaultValue={data.parameters.position}
                      disabled={!editingEnabled}
                      onChange={(v) => setPrimitiveParameter('primitive', data.uuid, 'position', v)}
                      bordered={false}
                      style={{ backgroundColor: 'rgba(255,255,255,0.2)' }} />
                  </Col>
                </Row>
                <Row align="middle" style={fieldStyle}>
                  <Col flex={2}>Speed:</Col>
                  <Col flex={3} style={{ textAlign: 'right' }}>
                    <InputNumber
                      min={20}
                      max={150}
                      size='small'
                      suffix='mm/s'
                      defaultValue={data.parameters.speed}
                      disabled={!editingEnabled}
                      onChange={(v) => setPrimitiveParameter('primitive', data.uuid, 'speed', v)}
                      bordered={false}
                      style={{ backgroundColor: 'rgba(255,255,255,0.2)' }} />
                  </Col>
                </Row>
              </>
            )}
            {settingsExpanded && data.type === 'delay' && (
              <>
                <Row align="middle" style={fieldStyle}>
                  <Col flex={2}>Duration:</Col>
                  <Col flex={3} style={{ textAlign: 'right' }}>
                    <InputNumber
                      min={0}
                      max={5}
                      formatter={(v) => (`${v} sec`)}
                      defaultValue={data.parameters.duration}
                      disabled={!editingEnabled}
                      onChange={(v) => setPrimitiveParameter('primitive', data.uuid, 'duration', v)}
                      bordered={false}
                      style={{ backgroundColor: 'rgba(255,255,255,0.2)' }} />
                  </Col>
                </Row>
              </>
            )}
            {settingsExpanded && data.type === 'move-trajectory' && (
              <>
                <Row align="middle" style={fieldStyle}>
                  <Col flex={2}>Velocity:</Col>
                  <Col flex={3} style={{ textAlign: 'right' }}>
                    <InputNumber
                      min={0.01}
                      max={5}
                      size='small'
                      suffix='mm'
                      defaultValue={data.parameters.velocity}
                      disabled={!editingEnabled}
                      onChange={(v) => setPrimitiveParameter('primitive', data.uuid, 'velocity', v)}
                      bordered={false}
                      style={{ backgroundColor: 'rgba(255,255,255,0.2)' }} />
                  </Col>
                </Row>
                <Row align="middle" style={fieldStyle}>
                  <Col span="8">Motion Type:</Col>
                  <Col span="16" style={{ textAlign: 'right' }}>
                    {data.parameters.move_type === 'ee_ik' ? (
                      <span style={selectionStyle}>IK</span>
                    ) : (
                      <Button type='text' disabled={!editingEnabled} onClick={() => setPrimitiveParameter('primitive', data.uuid, 'move_type', 'ee_ik')}>IK</Button>
                    )}
                    {data.parameters.move_type === 'joint' ? (
                      <span style={selectionStyle}>Joint</span>
                    ) : (
                      <Button type='text' disabled={!editingEnabled} onClick={() => setPrimitiveParameter('primitive', data.uuid, 'move_type', 'joint')}>Joint</Button>
                    )}
                  </Col>
                </Row>
              </>
            )}
          </animated.div>
        </div>
      }
      {['machine-initialize', 'process-start', 'process-wait', 'process-stop'].includes(data.type) && (
        <Row align="middle" style={fieldStyle}>
          <Col flex={2} style={{ paddingRight: 5 }}>Machine:</Col>
          <Col flex={3}>
            <NodeZone
              parentId={data.uuid}
              field='machine'
              ancestors={parameterAncestors.machine}
              context={context}
              uuid={data.parameters.machine}
              dragDisabled={!editingEnabled}
            />
          </Col>
        </Row>
      )}
      {data.type === 'move-trajectory' && (
        <Row align="middle" style={fieldStyle}>
          <Col flex={2} style={{ paddingRight: 5 }}>Trajectory:</Col>
          <Col flex={3}>
            <NodeZone
              parentId={data.uuid}
              field='trajectory'
              ancestors={parameterAncestors.trajectory}
              context={context}
              uuid={data.parameters.trajectory}
              dragDisabled={!editingEnabled}
            />
          </Col>
        </Row>
      )}
      {data.type === 'move-unplanned' && (
        <Row align="middle" style={fieldStyle}>
          <Col flex={2} style={{ paddingRight: 5 }}>To Location:</Col>
          <Col flex={3}>
            <NodeZone
              parentId={data.uuid}
              field='location'
              ancestors={parameterAncestors.location}
              context={context}
              uuid={data.parameters.location}
              dragDisabled={!editingEnabled}
            />
          </Col>
        </Row>
      )}
      {data.type === 'gripper' && (
        <Row align="middle" style={fieldStyle}>
          <Col flex={2} style={{ paddingRight: 5 }}>Thing:</Col>
          <Col flex={3}>
            <NodeZone
              parentId={data.uuid}
              field='thing'
              ancestors={parameterAncestors.thing}
              context={context}
              uuid={data.parameters.thing}
              dragDisabled={!editingEnabled}
            />
          </Col>
        </Row>
      )}
    </div>
  );
};
