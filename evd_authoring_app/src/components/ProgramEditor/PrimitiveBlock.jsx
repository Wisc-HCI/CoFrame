import React, { forwardRef, useCallback, useState } from "react";
import { StaticSortable } from './Wrappers';
import { NodeZone } from "./NodeZone";
import { ItemSortable } from "./Wrappers";
import { InputNumber, Row, Col, Button } from "antd";
import useStore from "../../stores/Store";
import blockStyles from "./blockStyles";
import { ReactComponent as PrimitiveIcon } from '../CustomIcons/Primitive.svg';
import { ReactComponent as SkillIcon } from '../CustomIcons/Skill.svg';
import { ReactComponent as ContainerIcon } from '../CustomIcons/Container.svg';
import Icon, { UnlockOutlined, LockOutlined, DownOutlined, RightOutlined } from '@ant-design/icons';
import './highlight.css';

export const PrimitiveBlock = forwardRef(({data,style,preview,ancestors,context}, ref) => {
  const { uuid, parameters } = data;
  
  const [focused, machine, thing, trajectory, location] = useStore(useCallback(state => {
    let machine = null;
    let thing = null;
    let trajectory = null;
    let location = null;

    // If there is a param and it exists in the context, create a fake data object, otherwise use the value in the store, if it exists there.
    if (parameters.machine_uuid && context[parameters.machine_uuid].real) {
      machine = state.data.machines[parameters.machine_uuid]
    } else if (parameters.machine_uuid && !context[parameters.machine_uuid].real) {
      machine = {uuid:parameters.machine_uuid,...context[parameters.machine_uuid]}
    }

    if (parameters.thing_uuid && context[parameters.thing_uuid].real) {
      thing = state.data.placeholders[parameters.thing_uuid]
    } else if (parameters.thing_uuid && !context[parameters.thing_uuid].real) {
      thing = {uuid:parameters.thing_uuid,pending_node:context[parameters.thing_uuid]}
    }

    if (parameters.trajectory_uuid && context[parameters.trajectory_uuid].real) {
      trajectory = state.data.trajectories[parameters.trajectory_uuid]
    } else if (parameters.trajectory_uuid && !context[parameters.trajectory_uuid].real) {
      trajectory = {uuid:parameters.trajectory_uuid,...context[parameters.trajectory_uuid]}
    }

    if (parameters.location_uuid && context[parameters.location_uuid].real) {
      location = state.data.locations[parameters.location_uuid]
    } else if (parameters.location_uuid && !context[parameters.location_uuid].real) {
      location = {uuid:parameters.location_uuid,...context[parameters.location_uuid]}
    }

    return [
    state.focusItem.uuid === uuid,
    machine,
    thing,
    trajectory,
    location,
  ]}, [uuid, parameters, context]));

  const [frame, clearFocusItem, focusExists] = useStore(state => [state.frame, state.clearFocusItem, state.focusItem.type !== null]);
  const unfocused = focusExists && !focused;

  const inDrawer = ancestors[0].uuid === 'drawer';
  const editingEnabled = !inDrawer && data.editable;
  const primitiveAncestors = [
    { uuid: data.uuid, accepts: ['uuid-waypoint'] },
    ...ancestors
  ];

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

  const [setPrimitiveParameter,moveTrajectoryBlock] = useStore(
    (state) => [state.setPrimitiveParameter,state.moveTrajectoryBlock]
  );

  // const trajectoryUuids = useStore(state=>Object.keys(state.data.trajectories));
  // const localOnly = Object.keys(context).filter(uuid=>trajectoryUuids.indexOf(uuid) < 0)

  let Glyph = null;
  if (data.type === 'node.primitive.skill-call.') {
    Glyph = SkillIcon;
  } else if (data.type === 'node.primitive.hierarchical.') {
    Glyph = ContainerIcon;
  } else {
    Glyph = PrimitiveIcon;
  }

  return (
    <div ref={preview} style={{ ...style, ...styles}} className={focused ? `focus-${frame}` : null} onClick={(e) => { e.stopPropagation(); unfocused && clearFocusItem() }}>
      <Row style={{ fontSize: 16, marginBottom: 7}} align='middle' justify='space-between'>
          <Col ref={ref} span={17} style={{textAlign:'left',backgroundColor:'rgba(255,255,255,0.1)',borderRadius:3,padding:4,cursor:'grab'}}>
              <Icon style={{marginLeft:4}} component={Glyph} />{' '}{data.name}
          </Col>
          <Col span={6} offset={1} style={{textAlign:'end'}}>
              {editingEnabled ? <UnlockOutlined /> : <LockOutlined />}
              {/* <Button
                  type='text'
                  style={{marginLeft:2}}
                  onClick={(e) => {e.stopPropagation();setFocusItem('program', uuid)}}
                  icon={<EllipsisOutlined />}
              /> */}
          </Col>
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
          <Col flex={2} style={{paddingRight:5}}>Machine:</Col>
          <Col flex={3}>
            <NodeZone
              ancestors={ancestors}
              onDrop={(dropData) => setPrimitiveParameter('primitive',data.uuid,'machine_uuid',dropData.uuid)}
              emptyMessage='No Machine'
              enabled={true}
            >
                {machine && (
                    <StaticSortable 
                        key={machine.uuid} 
                        id={machine.uuid} 
                        idx={0} 
                        ancestors={primitiveAncestors} 
                        itemType='uuid' 
                        context={context} 
                        data={{...machine,itemType:'machine',type:`uuid-machine`}}
                        // onMove={(dropData)=>moveTrajectoryWaypoint(dropData,uuid,idx)}
                        disabled={!editingEnabled}
                        onDelete={(_) => setPrimitiveParameter('primitive',data.uuid,'machine_uuid',null)}
                    />
                  )
              }
            </NodeZone>
          </Col>
        </Row>
      )}
      {data.type.includes("node.primitive.move-trajectory") && (
        <Row align="middle" style={fieldStyle}>
          <Col flex={2} style={{paddingRight:5}}>Trajectory:</Col>
          <Col flex={3}>
            <NodeZone
              ancestors={[{uuid,accepts:['uuid-trajectory','node.trajectory.']},...ancestors]}
              onMove={(dropData) => {
                if (context[dropData.uuid].real) {
                  moveTrajectoryBlock(dropData,data.uuid,null)
                } else {
                  setPrimitiveParameter('primitive',data.uuid,'trajectory_uuid',dropData.uuid)
                }
              }}
              emptyMessage='No Trajectory'
              enabled={editingEnabled}
            >
              {parameters.trajectory_uuid && (
                context[parameters.trajectory_uuid].real ? (
                  <ItemSortable 
                    id={data.parameters.trajectory_uuid} 
                    idx={0} 
                    ancestors={[{uuid,accepts:['uuid-trajectory','node.trajectory.']},...ancestors]} 
                    itemType='trajectory' 
                    context={context} 
                    onMove={(dropData) => {
                      if (context[dropData.uuid].real) {
                        moveTrajectoryBlock(dropData,data.uuid,null)
                      } else {
                        setPrimitiveParameter('primitive',data.uuid,'trajectory_uuid',dropData.uuid)
                      }
                    }}
                    disabled={!editingEnabled}/>
                ) : (
                  <StaticSortable 
                        key={trajectory.uuid} 
                        id={trajectory.uuid} 
                        idx={0} 
                        ancestors={primitiveAncestors} 
                        itemType='uuid' 
                        context={context} 
                        data={{...trajectory,itemType:'trajectory',type:`uuid-trajectory`}}
                        onDelete={(_) => setPrimitiveParameter('primitive',data.uuid,'trajectory_uuid',null)}
                        // onMove={(dropData)=>moveTrajectoryWaypoint(dropData,uuid,idx)}
                        disabled={!editingEnabled}
                    />
                )
              )}
            </NodeZone>
          </Col>
        </Row>
      )}
      {data.type.includes("node.primitive.move-unplanned") && (
        <Row align="middle" style={fieldStyle}>
          <Col flex={2} style={{paddingRight:5}}>To Location:</Col>
          <Col flex={3}>
            <NodeZone
              ancestors={ancestors}
              onDrop={(dropData) => setPrimitiveParameter('primitive',data.uuid,'location_uuid',dropData.uuid)}
              emptyMessage='No Location'
              enabled={true}
            >
                {location && (
                    <StaticSortable 
                        key={location.uuid} 
                        id={location.uuid} 
                        idx={0} 
                        ancestors={primitiveAncestors} 
                        itemType='uuid' 
                        context={context} 
                        data={{...location,itemType:'location',type:`uuid-location`}}
                        onDelete={(_) => setPrimitiveParameter('primitive',data.uuid,'location_uuid',null)}
                    
                        // onMove={(dropData)=>moveTrajectoryWaypoint(dropData,uuid,idx)}
                        disabled={!editingEnabled}
                    />
                  )
              }
            </NodeZone>
          </Col>
        </Row>
      )}
      {data.type.includes("node.primitive.gripper") && (
        <Row align="middle" style={fieldStyle}>
          <Col flex={2} style={{paddingRight:5}}>Thing:</Col>
          <Col flex={3}>
            <NodeZone
              ancestors={ancestors}
              onDrop={(dropData) => setPrimitiveParameter('primitive',data.uuid,'thing_uuid',dropData.uuid)}
              emptyMessage='No Thing'
              enabled={true}
            >
                {thing && (
                    <StaticSortable 
                        key={thing.uuid} 
                        id={thing.uuid} 
                        idx={0} 
                        ancestors={primitiveAncestors} 
                        itemType='uuid' 
                        context={context} 
                        data={{...thing,itemType:'placeholder',type:`uuid-thing`}}
                        onDelete={(_) => setPrimitiveParameter('primitive',data.uuid,'thing_uuid',null)}
                    
                        // onMove={(dropData)=>moveTrajectoryWaypoint(dropData,uuid,idx)}
                        disabled={!editingEnabled}
                    />
                  )
              }
            </NodeZone>
          </Col>
        </Row>
      )}
    </div>
  );
});
