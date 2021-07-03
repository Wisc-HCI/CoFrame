import React, {useCallback,useState} from 'react';



import { Empty } from 'antd';

import useEvdStore from '../../stores/EvdStore';
import useGuiStore from '../../stores/GuiStore';


import { List, Space, Button, Popover,InputNumber,Divider,Col,Input,Drawer,Form } from 'antd';
import { DeleteOutlined, EllipsisOutlined,EditOutlined } from '@ant-design/icons';
import {eulerFromQuaternion, quaternionFromEuler} from './Geometry';
import OrientationInput from './OrientationInput';
import PositionInput from './PositionInput';
import {MachineRegion} from './MachineRegion'

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
    let type = 'region'
    let searchTerm = 'region'
    const RegionUUIDS = useEvdStore(useCallback(state => Object.keys(state.data[type + 's'])
      .filter(Regionuuid => {
          return state.data[type + 's'][Regionuuid].name.toLowerCase().includes(searchTerm.toLowerCase())
      }), [type, searchTerm])
    )


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
      <Form labelCol = {{span : 4}} wrapperCol={{span : 100}} layout = 'horizontal' >

      <Form.Item label="Time">
      <Input style={{width:'10%'}} value={machine.process_time} disabled = 'true'>
      </Input>
      </Form.Item>

      <br/>
      <Form.Item label = "Input">
      <Input style={{width:'80%'}}/>
      </Form.Item>
      <br/>
      <Form.Item label = "Output">
      <Input style={{width:'80%'}}/>
      </Form.Item>
      </Form>

  <Divider orientation="left" style={{color:'white',borderTopColor:'rgba(255,255,255,0.12)',lineHeight: '1.5715px',paddingTop:'20px',paddingBottom:'5px'}}>
  Placement:
  </Divider>

  <div style={{display:'flex',flexDirection:'column'}}>
  <PositionInput value={[machine.pose_offset.position.x, machine.pose_offset.position.y,machine.pose_offset.position.z]}
  onChange={e=>setItemProperty('machine',machine.uuid,'machine',{...machine.pose_offset.position, x :e[0],y : e[1],z: e[2]})}/>
  <OrientationInput value={[machine.pose_offset.orientation.w, machine.pose_offset.orientation.x, machine.pose_offset.orientation.y, machine.pose_offset.orientation.z]}
  onChange={e=>setItemProperty('machine',machine.uuid,'orientation',{...machine.pose_offset.orientation, w:e[0],x :e[1],y : e[2],z: e[3]})}/>
  </div>

  <Button onClick={()=> setSecondaryFocusItem('regions',RegionUUIDS)}>
  Region

  </Button>



  </>





  )
}
