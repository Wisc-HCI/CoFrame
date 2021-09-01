import React, { forwardRef, useCallback, useState } from "react";
import { ParameterZone } from "./ParameterZone";
import useStore from "../../stores/Store";
import blockStyles from "./blockStyles";
import { StaticSortable } from './Wrappers';
import { NodeZone } from "./NodeZone";
import Icon, { UnlockOutlined, LockOutlined, DownOutlined, RightOutlined, EllipsisOutlined } from '@ant-design/icons';
import { Row, Col, Button, InputNumber } from 'antd';
import { ReactComponent as ContainerIcon } from '../CustomIcons/Container.svg'
import './highlight.css';

export const TrajectoryBlock = forwardRef(({data,ancestors,preview,style,context}, ref) => {
  const { uuid, start_location_uuid, end_location_uuid } = data;
  const focused = useStore(useCallback(state => state.focusItem.uuid === uuid, [uuid]));
  const [
    frame, clearFocusItem, setItemProperty, setFocusItem,
    moveTrajectoryWaypoint,focusExists] = useStore(state => [
      state.frame, 
      state.clearFocusItem, 
      state.setItemProperty,
      state.setFocusItem,
      state.moveTrajectoryWaypoint,
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

  const trajectoryAncestors = [
    { uuid: data.uuid, accepts: ['uuid-waypoint'] },
    ...ancestors
  ];

  const fieldStyle = {
    borderRadius: 4,
    margin: '1pt 1pt 4pt 1pt',
    padding: 5,
    backgroundColor: "rgba(0,0,0,0.1)"
  }

  const selectionStyle = {
    backgroundColor:'rgba(255,255,255,0.2)',
    borderRadius:2,
    margin:'0pt 8pt 12pt 0pt',
    padding: '6pt 15pt',
    fontSize: 14,
    height: 36
  }

  return (
    <div ref={preview} style={{ ...style, ...styles }} className={focused ? `focus-${frame}` : null} onClick={(e) => { e.stopPropagation(); unfocused && clearFocusItem() }}>
      <Row style={{ fontSize: 16, marginBottom: 7 }} align='middle' justify='space-between'>
          <Col ref={ref} span={17} style={{backgroundColor:'rgba(255,255,255,0.1)',borderRadius:3,padding:4,textAlign:'start'}}>
              <Icon style={{marginLeft:4}} component={ContainerIcon} />{' '}{data.name}
          </Col>
          <Col span={6} offset={1} style={{textAlign:'end'}}>
              {editingEnabled ? <UnlockOutlined /> : <LockOutlined />}
              <Button
                  type='text'
                  style={{marginLeft:2}}
                  onClick={(e) => {e.stopPropagation();setFocusItem('trajectory', data.uuid)}}
                  icon={<EllipsisOutlined />}
              />
          </Col>
      </Row>
      <div style={fieldStyle}>
        <Row align="middle" style={{marginBottom:5}}>
          
          <Col span="20">Settings:</Col>
          <Col span="4">
            <Button 
              disabled={inDrawer}
              onClick={()=>setSettingsExpanded(!settingsExpanded)}
              type='text' 
              icon={settingsExpanded ? (
                <DownOutlined/>
              ) : (
                <RightOutlined/>
              )}
            />
          </Col>
        </Row>
        {settingsExpanded && (
          <>
            <Row align="middle" style={fieldStyle}>
              <Col span="8">Motion Type:</Col>
              <Col span="16" style={{textAlign:'right'}}>
                {data.move_type==='ee_ik' ? (
                  <span style={selectionStyle}>IK</span>
                ) : (
                  <Button type='text' disabled={!editingEnabled} onClick={()=>setItemProperty('trajectory',uuid,'move_type','ee_ik')}>IK</Button>
                )}
                {data.move_type==='joint' ? (
                  <span style={selectionStyle}>Joint</span>
                ) : (
                  <Button type='text' disabled={!editingEnabled} onClick={()=>setItemProperty('trajectory',uuid,'move_type','joint')}>IK</Button>
                )}
              </Col>
            </Row>
            <Row align="middle" style={fieldStyle}>
              <Col span="8">Speed:</Col>
              <Col span="16" style={{textAlign:'right'}}>
                <InputNumber 
                  min={0.01} 
                  max={5} 
                  size='small'
                  defaultValue={data.velocity} 
                  disabled={!editingEnabled}
                  onChange={(v)=>setItemProperty('trajectory',uuid,'move_type',v)}
                  bordered={false} 
                  style={{backgroundColor:'rgba(255,255,255,0.2)'}}/>
              </Col>
            </Row>
          </>
        )}
      </div>
      <Row align="middle" style={fieldStyle}>
        <Col span="8">Start Location:</Col>
        <Col span="16">
          <ParameterZone
            displayText={context[start_location_uuid]}
            acceptTypes={['uuid-location']}
            itemType="location"
            canRemove={editingEnabled}
            onRemove={() => setItemProperty('trajectory', uuid, 'start_location_uuid', null)}
            onDrop={(data) => setItemProperty('trajectory', uuid, 'start_location_uuid', data.uuid)}
          />
        </Col>
      </Row>
      <Row align="middle" style={fieldStyle}>
        <Col span="8">Waypoints:</Col>
        <Col span="16">
          <NodeZone
            ancestors={ancestors}
            onDrop={(dropData) => moveTrajectoryWaypoint(dropData,uuid,0)}
            emptyMessage='No Waypoints'
            enabled={true}
          >
              {data.waypoint_uuids.map((id,idx)=>(
                <div key={id} style={{marginBottom:idx===data.waypoint_uuids.length-1?0:3}}>
                  <StaticSortable 
                      key={id} 
                      id={id} 
                      idx={idx} 
                      ancestors={trajectoryAncestors} 
                      itemType='uuid' 
                      context={context} 
                      data={{itemType:'waypoint',uuid:id,type:`uuid-waypoint`}}
                      onMove={(dropData)=>moveTrajectoryWaypoint(dropData,uuid,idx)}
                      disabled={!editingEnabled}
                  />
                </div>
                  
              ))}
          </NodeZone>
        </Col>
      </Row>
      <Row align="middle" style={fieldStyle}>
        <Col span="8">End Location:</Col>
        <Col span="16">
          <ParameterZone
            displayText={context[end_location_uuid]}
            acceptTypes={['uuid-location']}
            itemType="location"
            canRemove={editingEnabled}
            onRemove={() => setItemProperty('trajectory', uuid, 'end_location_uuid', null)}
            onDrop={(data) => setItemProperty('trajectory', uuid, 'end_location_uuid', data.uuid)}
          />
        </Col>
      </Row>
    </div>
  );
});