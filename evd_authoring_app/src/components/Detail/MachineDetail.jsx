import React, { useCallback } from 'react';

import useEvdStore from '../../stores/EvdStore';

import { Space, Divider, Row, Input } from 'antd';
import OrientationInput from './OrientationInput';
import PositionInput from './PositionInput';
import { MachineInOutTypeDetail } from './MachineInOutTypeDetail';


export const MachineDetail = ({ uuid }) => {

  const { machine } = useEvdStore(useCallback(state => ({
    machine: state.data.machines[uuid],

  })
    , [uuid]))
  const { setItemProperty } = useEvdStore(state => ({
    setItemProperty: state.setItemProperty
  }));

  const { TextArea } = Input;

  return (
    <>

      <Space />

      <TextArea
        defaultValue={machine.description}
        disabled={!machine.editable}
      />

      <Divider orientation="left" style={{ color: 'white', borderTopColor: 'rgba(255,255,255,0.12)', lineHeight: '1.5715px', paddingTop: '20px', paddingBottom: '5px' }}>
        Processing:
      </Divider>

      <Row justify='space-between' style={{paddingRight:20,paddingTop:10,paddingLeft:20}}>
        <b>Time :</b>
        <Input style={{maxWidth:100}} value={machine.process_time} disabled={!machine.editable} suffix='sec'/>
      </Row>
      
      <MachineInOutTypeDetail machine={machine} input />
      <MachineInOutTypeDetail machine={machine} />

      <Divider orientation="left" style={{ color: 'white', borderTopColor: 'rgba(255,255,255,0.12)', lineHeight: '1.5715px', paddingTop: '20px', paddingBottom: '5px' }}>
        Placement:
      </Divider>

      <div style={{ display: 'flex', flexDirection: 'column' }}>
        <PositionInput value={[machine.pose_offset.position.x, machine.pose_offset.position.y, machine.pose_offset.position.z]}
          onChange={e => setItemProperty('machine', machine.uuid, 'machine', { ...machine.pose_offset.position, x: e[0], y: e[1], z: e[2] })} />
        <OrientationInput value={[machine.pose_offset.orientation.w, machine.pose_offset.orientation.x, machine.pose_offset.orientation.y, machine.pose_offset.orientation.z]}
          onChange={e => setItemProperty('machine', machine.uuid, 'orientation', { ...machine.pose_offset.orientation, w: e[0], x: e[1], y: e[2], z: e[3] })} />
      </div>
      <br />



    </>





  )
}
