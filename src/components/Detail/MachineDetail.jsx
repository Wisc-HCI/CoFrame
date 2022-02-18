import React, { useCallback } from 'react';

import useStore from '../../stores/Store';

//import { Space, Divider, Row, Input } from 'antd';
import { MachineInOutTypeDetail } from './MachineInOutTypeDetail';
import { Toggle } from '../Toggle';
import { TextArea, Text, Box, TextInput } from 'grommet';
import OrientationInput from './OrientationInput';
import PositionInput from './PositionInput';


export const MachineDetail = ({ uuid }) => {

  const machine = useStore(useCallback(state => state.data.machines[uuid], [uuid]));
  const setItemProperty = useStore(state => state.setItemProperty);

  //const { TextArea } = Input;

  return (
    <>

      <TextArea
        value={machine.description}
        disabled={!machine.editable}
      />

      <Box orientation="left" style={{ color: 'white', borderTopColor: 'rgba(255,255,255,0.12)', lineHeight: '1.5715px', paddingTop: '20px', paddingBottom: '5px' }}>
        <Text>
          Processing:
        </Text>

      </Box>

      <Box justify='space-between' style={{ paddingRight: 20, paddingTop: 10, paddingLeft: 20 }}>
        <Text>Time :</Text>
        <TextInput style={{ maxWidth: 100 }} value={machine.process_time} disabled={!machine.editable} suffix='sec' />
      </Box>

      <MachineInOutTypeDetail machine={machine} input />
      <MachineInOutTypeDetail machine={machine} />

      <Box orientation="left" style={{ color: 'white', borderTopColor: 'rgba(255,255,255,0.12)', lineHeight: '1.5715px', paddingTop: '20px', paddingBottom: '5px' }}>
        Placement:
      </Box>

      <div style={{ display: 'flex', flexDirection: 'column' }}>
        <PositionInput value={[machine.pose_offset.position.x, machine.pose_offset.position.y, machine.pose_offset.position.z]}
          onChange={e => setItemProperty('machine', machine.uuid, 'position', { ...machine.pose_offset.position, x: e[0], y: e[1], z: e[2] })} />
        <OrientationInput value={[machine.pose_offset.orientation.w, machine.pose_offset.orientation.x, machine.pose_offset.orientation.y, machine.pose_offset.orientation.z]}
          onChange={e => setItemProperty('machine', machine.uuid, 'orientation', { ...machine.pose_offset.orientation, w: e[0], x: e[1], y: e[2], z: e[3] })} />
      </div>
      <br />



    </>





  )
}
