import React, {useCallback,useState} from 'react';



import { Empty } from 'antd';

import useEvdStore from '../../stores/EvdStore';


import { List, Space, Button, Popover,InputNumber,Divider,Col,Input } from 'antd';
import { DeleteOutlined, EllipsisOutlined,EditOutlined } from '@ant-design/icons';
import {eulerFromQuaternion, quaternionFromEuler} from './Geometry';
import OrientationInput from './OrientationInput';
import PositionInput from './PositionInput';

export const MachineDetail = ({uuid}) => {

    const {machine} = useEvdStore(useCallback(state=>({
        machine:state.data.machines[uuid],

    })
      ,[uuid]))
      const {setItemProperty} = useEvdStore(state=>({
          setItemProperty:state.setItemProperty
      }));

    const { TextArea } = Input;

    // const { deleteItem, setItemProperty } = useEvdStore(state=>({
    //     deleteItem:state.deleteItem,
    //     setItemProperty:state.setItemProperty
    // }));

    console.log(machine);

    return (
      <>
      
      <Space/>

      <TextArea
        defaultValue={machine.description}
        disabled = {!machine.editable}
      />

      <Divider orientation="left" style={{color:'white',borderTopColor:'rgba(255,255,255,0.12)',lineHeight: '1.5715px',paddingTop:'20px',paddingBottom:'5px'}}>
      Processing:
      </Divider>
      <br/>
      <div style={{top:'100px',display:'flex',flexDirection:'column'}}>
      <div style={{ display:'flex',justifyContent: 'space-between',alignItems:'center'}}>
      Time:  <Input style={{left:'-240px',width:'10%'}} value={machine.process_time} disabled = 'true'>


      </Input>



      </div>
      <br/>
      <div style={{ display:'flex',justifyContent: 'space-between',alignItems:'center'}}>
      input: <Input style={{width:'80%'}}/>
      </div>
      <br/>
      <div style={{ display:'flex',justifyContent: 'space-between',alignItems:'center'}}>
      output: <Input style={{width:'80%'}}/>
      </div>
      </div>

  <Divider orientation="left" style={{color:'white',borderTopColor:'rgba(255,255,255,0.12)',lineHeight: '1.5715px',paddingTop:'20px',paddingBottom:'5px'}}>
  Placement:
  </Divider>

  <div style={{display:'flex',flexDirection:'column'}}>
  <PositionInput value={[machine.pose_offset.position.x, machine.pose_offset.position.y,machine.pose_offset.position.z]}
  onChange={e=>setItemProperty('machine',machine.uuid,'machine',{...machine.pose_offset.position, x :e[0],y : e[1],z: e[2]})}/>
  <OrientationInput value={[machine.pose_offset.orientation.w, machine.pose_offset.orientation.x, machine.pose_offset.orientation.y, machine.pose_offset.orientation.z]}
  onChange={e=>setItemProperty('machine',machine.uuid,'orientation',{...machine.pose_offset.orientation, w:e[0],x :e[1],y : e[2],z: e[3]})}/>
  </div>
  </>





  )
}
