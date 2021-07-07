import React, {useCallback,useState} from 'react';



import { Empty } from 'antd';

import useEvdStore from '../../stores/EvdStore';
import useGuiStore from '../../stores/GuiStore';


import { List, Space, Button, Popover,InputNumber,Divider,Col,Input,Drawer,Form,Card } from 'antd';
import { DeleteOutlined, EllipsisOutlined,EditOutlined } from '@ant-design/icons';
import {eulerFromQuaternion, quaternionFromEuler} from './Geometry';
import OrientationInput from './OrientationInput';
import PositionInput from './PositionInput';
import {MachineRegion} from './MachineRegion';
import {MachineInOutTypeDetail} from './MachineInOutTypeDetail';


export const MachineDetail = ({uuid}) => {

    const {machine} = useEvdStore(useCallback(state=>({
        machine:state.data.machines[uuid],

    })
      ,[uuid]))
      const {setItemProperty} = useEvdStore(state=>({
          setItemProperty:state.setItemProperty
      }));

    const { TextArea } = Input;

    const {secondaryFocusItem,setSecondaryFocusItem,clearSecondaryFocusItem} = useGuiStore(
      state => ({
        secondaryFocusItem : state.secondaryFocusItem,
        clearSecondaryFocusItem : state.clearFocusItem,
        setSecondaryFocusItem: state.setSecondaryFocusItem
      })
    )

//1, multiple region id, which one to use for the current machine?
// answer:
//2. how to pass in props into MachineRegion ?
//3. quantity?
//4.









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

      <Card>
       <b style = {{paddingRight : '10px'}}>Time :</b> <Input style={{width:'12%'}} value={machine.process_time} disabled = 'true'>
      </Input>
      <br/>
      <br/>

       <b style = {{paddingRight : '10px'}}>Input :</b> <MachineInOutTypeDetail machine = {machine} input = "true"/>


        <b style = {{paddingRight : '10px'}}>Output :</b> <MachineInOutTypeDetail machine = {machine} input = "false"/>

      </Card>

  <Divider orientation="left" style={{color:'white',borderTopColor:'rgba(255,255,255,0.12)',lineHeight: '1.5715px',paddingTop:'20px',paddingBottom:'5px'}}>
  Placement:
  </Divider>

  <div style={{display:'flex',flexDirection:'column'}}>
  <PositionInput value={[machine.pose_offset.position.x, machine.pose_offset.position.y,machine.pose_offset.position.z]}
  onChange={e=>setItemProperty('machine',machine.uuid,'machine',{...machine.pose_offset.position, x :e[0],y : e[1],z: e[2]})}/>
  <OrientationInput value={[machine.pose_offset.orientation.w, machine.pose_offset.orientation.x, machine.pose_offset.orientation.y, machine.pose_offset.orientation.z]}
  onChange={e=>setItemProperty('machine',machine.uuid,'orientation',{...machine.pose_offset.orientation, w:e[0],x :e[1],y : e[2],z: e[3]})}/>
  </div>
  <br/>



  </>





  )
}
