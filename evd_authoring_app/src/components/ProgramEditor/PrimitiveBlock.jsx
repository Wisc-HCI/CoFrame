import React, { forwardRef, useCallback, useState } from "react";
import { ParameterZone } from "./ParameterZone";
import { NodeZone } from "./NodeZone";
import { ItemSortable } from "./Wrappers";
import { InputNumber, Row, Col, Button, Tag } from "antd";
import useEvdStore from "../../stores/EvdStore";
import useGuiStore from "../../stores/GuiStore";
import blockStyles from "./blockStyles";
import { ReactComponent as PrimitiveIcon } from '../CustomIcons/Primitive.svg';
import { ReactComponent as SkillIcon } from '../CustomIcons/Skill.svg';
import { ReactComponent as ContainerIcon } from '../CustomIcons/Container.svg';
import Icon, { UnlockOutlined, LockOutlined, DownOutlined, RightOutlined } from '@ant-design/icons';
import './highlight.css';

export const PrimitiveBlock = forwardRef(({data,style,preview,ancestors,context}, ref) => {
  const { uuid } = data;
  const focused = useGuiStore(useCallback(state => state.focusItem.uuid === uuid, [uuid]));
  const focusExists = useGuiStore(state => state.focusItem.type !== null);
  const [frame, clearFocusItem] = useGuiStore(state => [state.frame, state.clearFocusItem]);
  const unfocused = focusExists && !focused;

  const inDrawer = ancestors[0].uuid === 'drawer';
  const editingEnabled = !inDrawer && data.editable;

  const [settingsExpanded, setSettingsExpanded] = useState(false);

  const primitivesWithSettings = [
    'node.primitive.gripper.',
    'node.primitive.delay.'
  ]

  const styles = {
    backgroundColor:
      blockStyles[data.type],
    minHeight: 30,
    minWidth: 250,
    borderRadius: 3,
    fontSize: 14,
    margin: 4,
    padding: 5,
    position: 'relative',
    zIndex: focused ? 100 : 1
  };

  const fieldStyle = {
    borderRadius: 4,
    margin: '1pt 1pt 4pt 1pt',
    padding: 5,
    backgroundColor: "rgba(0,0,0,0.1)"
  }

  const [setPrimitiveParameter,moveTrajectoryBlock] = useEvdStore(
    (state) => [state.setPrimitiveParameter,state.moveTrajectoryBlock]
  );

  const trajectoryUuids = useEvdStore(state=>Object.keys(state.data.trajectories));
  const localOnly = Object.keys(context).filter(uuid=>trajectoryUuids.indexOf(uuid) < 0)

  let Glyph = null;
  if (data.type === 'node.primitive.skill-call.') {
    Glyph = SkillIcon;
  } else if (data.type === 'node.primitive.hierarchical.') {
    Glyph = ContainerIcon;
  } else {
    Glyph = PrimitiveIcon;
  }

  return (
    <div ref={preview} style={{ ...style, ...styles }} className={focused ? `focus-${frame}` : null} onClick={(e) => { e.stopPropagation(); unfocused && clearFocusItem() }}>
      <Row ref={ref} style={{ fontSize: 16, marginBottom: 5 }} align='middle' justify='space-between'>
        <span>
          <Icon component={Glyph} />{' '}{data.name}
        </span>
        {editingEnabled ? <UnlockOutlined /> : <LockOutlined />}
      </Row>
      <div style={fieldStyle} hidden={primitivesWithSettings.indexOf(data.type) < 0}>
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
            {data.type === 'node.primitive.gripper.' && (
              <>
                <Row align="middle" style={fieldStyle}>
                  <Col flex={2}>Position:</Col>
                  <Col flex={3} style={{ textAlign: 'right' }}>
                    <InputNumber
                      min={0}
                      max={5}
                      size='small'
                      defaultValue={data.parameters.position}
                      disabled={!editingEnabled}
                      onChange={(v) => setPrimitiveParameter('primitive', data.uuid, 'position', v)}
                      bordered={false}
                      style={{ backgroundColor: 'rgba(255,255,255,0.2)' }} />
                  </Col>
                </Row>
                <Row align="middle" style={fieldStyle}>
                  <Col flex={2}>Effort:</Col>
                  <Col flex={3} style={{ textAlign: 'right' }}>
                    <InputNumber
                      min={0}
                      max={5}
                      size='small'
                      defaultValue={data.parameters.effort}
                      disabled={!editingEnabled}
                      onChange={(v) => setPrimitiveParameter('primitive', data.uuid, 'effort', v)}
                      bordered={false}
                      style={{ backgroundColor: 'rgba(255,255,255,0.2)' }} />
                  </Col>
                </Row>
                <Row align="middle" style={fieldStyle}>
                  <Col flex={2}>Speed:</Col>
                  <Col flex={3} style={{ textAlign: 'right' }}>
                    <InputNumber
                      min={0}
                      max={5}
                      size='small'
                      defaultValue={data.parameters.speed}
                      disabled={!editingEnabled}
                      onChange={(v) => setPrimitiveParameter('primitive', data.uuid, 'speed', v)}
                      bordered={false}
                      style={{ backgroundColor: 'rgba(255,255,255,0.2)' }} />
                  </Col>
                </Row>
              </>
            )}
            {data.type === 'node.primitive.delay.' && (
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
          </>
        )}
      </div>
      {data.type.includes("node.primitive.machine-primitive") && (
        <Row align="middle" style={fieldStyle}>
          <Col flex={2}>Machine:</Col>
          <Col flex={3}>
            <ParameterZone
              displayText={context[data.parameters.machine_uuid]}
              acceptTypes={['uuid-machine']}
              itemType="machine"
              canRemove={editingEnabled}
              onRemove={() => setPrimitiveParameter('primitive',data.uuid,'machine_uuid',null)}
              onDrop={(dropData) => setPrimitiveParameter('primitive',data.uuid,'machine_uuid',dropData.uuid)}
            />
          </Col>
        </Row>
      )}
      {data.type.includes("node.primitive.move-trajectory") && (
        <Row align="middle" style={fieldStyle}>
          <Col flex={2}>Trajectory:</Col>
          <Col flex={3}>
            <NodeZone
              ancestors={[{uuid,accepts:['uuid-trajectory','node.trajectory.']},...ancestors]}
              onMove={(dropData) => {
                if (localOnly.indexOf(dropData) >= 0) {
                  setPrimitiveParameter('primitive',data.uuid,'trajectory_uuid',dropData.uuid)
                } else {
                  moveTrajectoryBlock(dropData,data.uuid,null)
                }
              }}
              emptyMessage='No Trajectory'
              enabled={editingEnabled}
            >
              {data.parameters.trajectory_uuid && (
                trajectoryUuids.indexOf(data.parameters.trajectory_uuid) >= 0 ? (
                  <ItemSortable 
                    id={data.parameters.trajectory_uuid} 
                    idx={0} 
                    ancestors={[{uuid,accepts:['uuid-trajectory','node.trajectory.']},...ancestors]} 
                    itemType='trajectory' 
                    context={context} 
                    onMove={(dropData) => {
                      if (localOnly.indexOf(dropData) >= 0) {
                        setPrimitiveParameter('primitive',data.uuid,'trajectory_uuid',dropData.uuid)
                      } else {
                        moveTrajectoryBlock(dropData,data.uuid,null)
                      }
                    }}
                    disabled={!editingEnabled}/>
                ) : (
                  <Tag 
                    color={blockStyles['trajectory']} 
                    closable={editingEnabled} 
                    onClose={() => setPrimitiveParameter('primitive',data.uuid,'trajectory_uuid',null)} 
                    style={{width:'100%'}}>{context[data.parameters.trajectory_uuid]}</Tag>
                )
              )}
            </NodeZone>
          </Col>
        </Row>
      )}
      {data.type.includes("node.primitive.move-unplanned") && (
        <Row align="middle" style={fieldStyle}>
          <Col flex={2}>To Location:</Col>
          <Col flex={3}>
            <ParameterZone
              displayText={context[data.parameters.location_uuid]}
              acceptTypes={['uuid-location']}
              itemType="location"
              canRemove={editingEnabled}
              onRemove={() => setPrimitiveParameter('primitive',data.uuid,'location_uuid',null)}
              onDrop={(dropData) => setPrimitiveParameter('primitive',data.uuid,'location_uuid',dropData.uuid)}
            />
          </Col>
        </Row>
      )}
      {data.type.includes("node.primitive.gripper") && (
        <Row align="middle" style={fieldStyle}>
          <Col flex={2}>Thing:</Col>
          <Col flex={3}>
            <ParameterZone
              displayText={context[data.parameters.thing_uuid]}
              acceptTypes={['uuid-thing']}
              itemType="thing"
              canRemove={editingEnabled}
              onRemove={() => setPrimitiveParameter('primitive',data.uuid,'thing_uuid',null)}
              onDrop={(dropData) => setPrimitiveParameter('primitive',data.uuid,'thing_uuid',dropData.uuid)}
            />
          </Col>
        </Row>
      )}
    </div>
  );
});
