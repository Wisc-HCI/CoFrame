import React, { useCallback, useState } from "react";
import { NodeZone } from "./NodeZone";
// import { ItemSortable } from "./Wrappers";
import { InputNumber, Row, Col, Button } from "antd";
import useStore from "../../stores/Store";
import shallow from 'zustand/shallow';
import blockStyles from "./blockStyles";
import { ReactComponent as PrimitiveIcon } from '../CustomIcons/Primitive.svg';
import { ReactComponent as SkillIcon } from '../CustomIcons/Skill.svg';
import { ReactComponent as ContainerIcon } from '../CustomIcons/Container.svg';
import Icon, { UnlockOutlined, LockOutlined, DownOutlined, RightOutlined, EyeOutlined } from '@ant-design/icons';
import './highlight.css';
import { UUIDBlock } from "./UUIDBlock";
import { useSpring, animated } from '@react-spring/web';
import { config } from 'react-spring';
import { useDrag } from "react-dnd";
import { TrajectoryBlock } from "./TrajectoryBlock";
import useMeasure from "react-use-measure";

export const PrimitiveBlock = ({
  staticData, uuid, parentData, onDelete, dragBehavior,
  dragDisabled, ancestors, context, idx, after,
}) => {

  const [focused, data, parameters, executable] = useStore(useCallback(state => {
    let parameterValues = {};

    const data = staticData ? staticData : state.data[uuid] ? state.data[uuid] : {parameters:{},type:''};
    const executable = state.executablePrimitives[data.uuid] ? true : false;

    // If there is a param and it exists in the context, create a fake data object, otherwise use the value in the store, if it exists there.
    Object.keys(data.parameters).forEach(parameter=>{
      if (data.parameters[parameter] && state.data[data.parameters[parameter]]) {
        parameterValues[parameter] = {...state.data[data.parameters[parameter]],real:true}
      } else if (data.parameters[parameter] && context[data.parameters[parameter]]) {
        parameterValues[parameter] = { uuid: data.parameters[parameter], ...context[data.parameters[parameter]] }
      }
    })

    return [
      state.focusItem.uuid === uuid,
      data,
      parameterValues,
      executable
    ]
  }, [staticData, uuid, context]),shallow);

  const [frame, setFocusItem, clearFocusItem, focusExists, 
    setPrimitiveParameter, moveTrajectoryBlock,
    deletePrimitiveTrajectory] = useStore(
    (state) => [
      state.frame, state.setFocusItem, state.clearFocusItem, state.focusItem.type !== null, 
      state.setPrimitiveParameter, state.moveTrajectoryBlock, 
      state.deletePrimitiveTrajectory], shallow)

  const unfocused = focusExists && !focused;

  const inDrawer = ancestors[0].uuid === 'drawer';
  const editingEnabled = !inDrawer && data.editable;


  const [{ isDragging }, drag, preview] = useDrag(() => ({
    type: data.type,
    item: { ...data, parentData, dragBehavior, onDelete: onDelete && data.deleteable ? ()=>onDelete(data) : null, idx },
    options: { dragEffect: dragBehavior },
    canDrag: (_)=> !dragDisabled,
    collect: monitor => ({
      isDragging: monitor.isDragging()
    })
  }))


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
      { uuid: data.uuid, accepts: ['uuid-trajectory', 'node.trajectory.'] },
      ...ancestors
    ],
    thing: [
      { uuid: data.uuid, accepts: ['uuid-placeholder'] },
      ...ancestors
    ]
  }

  const [settingsRef, { height }] = useMeasure();
  const [settingsExpanded, setSettingsExpanded] = useState(false);
  const settingsStyle = useSpring({ height: height, config: config.stiff});
  
  const primitivesWithSettings = [
    'gripper',
    'delay'
  ]

  const styles = {
    backgroundColor:
      blockStyles[data.type],
    minHeight: 30,
    minWidth: 250,
    borderRadius: 3,
    fontSize: 14,
    padding: 5,
    position: 'relative',
    margin:0,
    zIndex: focused ? 100 : 1
  };

  const fieldStyle = {
    borderRadius: 4,
    margin: '1pt 1pt 4pt 1pt',
    padding: 5,
    backgroundColor: "rgba(0,0,0,0.1)"
  }

  let Glyph = null;
  if (data.type === 'skill-call') {
    Glyph = SkillIcon;
  } else if (data.type === 'hierarchical') {
    Glyph = ContainerIcon;
  } else {
    Glyph = PrimitiveIcon;
  }

  const parameterDrop = (dropData, parameter) => {
    if (dropData.parentData.uuid === uuid && dropData.dragBehavior === 'move') {
      console.log('move')
      setPrimitiveParameter('primitive', uuid, dropData.parentData.field, null);
      setPrimitiveParameter('primitive', uuid, parameter, dropData.uuid);
    } else {
      console.log('copy')
      setPrimitiveParameter('primitive', uuid, parameter, dropData.uuid);
    }
  }

  const trajectoryDrop = (dropData) => {
    if (dropData.type.includes('uuid')) {
      setPrimitiveParameter('primitive', uuid, 'trajectory', dropData.uuid);
    } else {
      console.log(dropData)
      moveTrajectoryBlock(dropData, uuid, 'trajectory')
    }
  }

  return (
    <div hidden={isDragging && dragBehavior==='move'}>
      <div ref={preview} style={styles} className={focused ? `focus-${frame}` : null} onClick={(e) => { e.stopPropagation(); unfocused && clearFocusItem() }}>
        <Row wrap={false} style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
          <Row wrap={false} align='middle' ref={dragDisabled ? null : drag} span={17} style={{ textAlign: 'left', backgroundColor: 'rgba(255,255,255,0.1)', borderRadius: 3, padding: 4, minWidth: 200, maxWidth: 230, cursor: dragDisabled ? 'not-allowed':'grab', zIndex:101 }}>
            <Icon style={{ marginLeft: 4, marginRight: 4 }} component={Glyph} />
            {data.name}
          </Row>
          <Row wrap={false} align='middle' style={{ textAlign: 'end' }}>
            {executable && <Button type='text' icon={<EyeOutlined/>} onClick={(e) => {e.stopPropagation();setFocusItem('primitive', uuid)}}/>}
            {editingEnabled ? <UnlockOutlined style={{marginRight:5,marginLeft:5}}/> : <LockOutlined style={{marginRight:5,marginLeft:5}}/>}
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
                    {/* <Row align="middle" style={fieldStyle}>
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
                    </Row> */}
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
              </div>
            </animated.div>
          </div>}
        {['machine-initialize', 'process-start', 'process-wait', 'process-stop'].includes(data.type) && (
          <Row align="middle" style={fieldStyle}>
            <Col flex={2} style={{ paddingRight: 5 }}>Machine:</Col>
            <Col flex={3}>
              <NodeZone
                ancestors={parameterAncestors.machine}
                context={context}
                onDrop={(dropData) => parameterDrop(dropData, 'machine')}
                emptyMessage='No Machine'
                dropDisabled={!editingEnabled}
              >
                {parameters.machine && (
                  <UUIDBlock
                    key={parameters.machine.uuid}
                    id={parameters.machine.uuid}
                    idx={0}
                    dragBehavior='move'
                    hoverBehavior='replace'
                    ancestors={parameterAncestors.machine}
                    context={context}
                    parentData={{ type: 'primitive', uuid, field: 'machine' }}
                    data={{ ...parameters.machine, itemType: 'machine', type: `uuid-machine` }}
                    onDelete={(_) => setPrimitiveParameter('primitive', uuid, 'machine', null)}
                    onDrop={(dropData) => parameterDrop(dropData, 'machine')}
                    dragDisabled={!editingEnabled}
                    dropDisabled={!editingEnabled}
                  />
                )
                }
              </NodeZone>
            </Col>
          </Row>
        )}
        {data.type === 'move-trajectory' && (
          <Row align="middle" style={fieldStyle}>
            <Col flex={2} style={{ paddingRight: 5 }}>Trajectory:</Col>
            <Col flex={3}>
              <NodeZone
                ancestors={parameterAncestors.trajectory}
                context={context}
                parentData={{ type: 'primitive', uuid, field: 'trajectory' }}
                onDelete={(_) => deletePrimitiveTrajectory(uuid, 'trajectory', parameters.trajectory.uuid)}
                onDrop={trajectoryDrop}
                emptyMessage='No Trajectory'
                dropDisabled={!editingEnabled}
              > 
                {parameters.trajectory && (
                  parameters.trajectory.real ? (
                    <TrajectoryBlock
                      uuid={parameters.trajectory.uuid}
                      idx={0}
                      parentData={{ type: 'primitive', uuid, field: 'trajectory' }}
                      ancestors={parameterAncestors.trajectory}
                      dragBehavior='move'
                      dragDisabled={!editingEnabled}
                      onDelete={(_) => deletePrimitiveTrajectory(uuid, 'trajectory', parameters.trajectory.uuid)}
                      context={context}
                    />
                  ) : (
                    <UUIDBlock
                      key={parameters.trajectory.uuid}
                      id={parameters.trajectory.uuid}
                      idx={0}
                      dragBehavior='move'
                      hoverBehavior='replace'
                      ancestors={parameterAncestors.trajectory}
                      context={context}
                      parentData={{ type: 'primitive', uuid, field: 'trajectory' }}
                      data={{ ...parameters.trajectory, itemType: 'trajectory', type: `uuid-trajectory` }}
                      onDelete={(_) => setPrimitiveParameter('primitive', uuid, 'trajectory', null)}
                      onDrop={trajectoryDrop}
                      dragDisabled={!editingEnabled}
                      dropDisabled={!editingEnabled}
                    />
                  )
                )}
              </NodeZone>
            </Col>
          </Row>
        )}
        {data.type === 'move-unplanned' && (
          <Row align="middle" style={fieldStyle}>
            <Col flex={2} style={{ paddingRight: 5 }}>To Location:</Col>
            <Col flex={3}>
              <NodeZone
                ancestors={parameterAncestors.location}
                context={context}
                onDrop={(dropData) => parameterDrop(dropData, 'location')}
                emptyMessage='No Location'
                dropDisabled={!editingEnabled}
              >
                {parameters.location && (
                  <UUIDBlock
                    key={parameters.location.uuid}
                    id={parameters.location.uuid}
                    idx={0}
                    dragBehavior='move'
                    hoverBehavior='replace'
                    ancestors={parameterAncestors.location}
                    context={context}
                    parentData={{ type: 'primitive', uuid, field: 'location' }}
                    data={{ ...parameters.location, itemType: 'location', type: `uuid-location` }}
                    onDelete={(_) => setPrimitiveParameter('primitive', uuid, 'location', null)}
                    onDrop={(dropData) => parameterDrop(dropData, 'location')}
                    dragDisabled={!editingEnabled}
                    dropDisabled={!editingEnabled}
                  />
                )
                }
              </NodeZone>
            </Col>
          </Row>
        )}
        {data.type === 'gripper' && (
          <Row align="middle" style={fieldStyle}>
            <Col flex={2} style={{ paddingRight: 5 }}>Thing:</Col>
            <Col flex={3}>
              <NodeZone
                ancestors={parameterAncestors.thing}
                context={context}
                onDrop={(dropData) => parameterDrop(dropData, 'thing')}
                emptyMessage='No Thing'
                dropDisabled={!editingEnabled}
              >
                {parameters.thing && (
                  <UUIDBlock
                    key={parameters.thing.uuid}
                    id={parameters.thing.uuid}
                    idx={0}
                    dragBehavior='move'
                    hoverBehavior='replace'
                    ancestors={parameterAncestors.thing}
                    context={context}
                    parentData={{ type: 'primitive', uuid, field: 'thing' }}
                    data={{ ...parameters.thing, itemType: 'placeholder', type: `uuid-placeholder` }}
                    onDelete={(_) => setPrimitiveParameter('primitive', uuid, 'thing', null)}
                    onDrop={(dropData) => parameterDrop(dropData, 'thing')}
                    dragDisabled={!editingEnabled}
                    dropDisabled={!editingEnabled}
                  />
                )
                }
              </NodeZone>
            </Col>
          </Row>
        )}
      </div>
      {!(isDragging && dragBehavior === 'move') && after}
    </div>
  );
};
