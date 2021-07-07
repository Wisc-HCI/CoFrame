import React, {useCallback,useState} from 'react';



import { Empty } from 'antd';

import useEvdStore from '../../stores/EvdStore';


import { List, Space, Button, Popover,InputNumber,Divider,Col,Input,Drawer } from 'antd';
import { DeleteOutlined, EllipsisOutlined,EditOutlined } from '@ant-design/icons';
import {eulerFromQuaternion, quaternionFromEuler} from './Geometry';
import OrientationInput from './OrientationInput';
import PositionInput from './PositionInput';

export const MachineRegion = (props) => {

  const {type} = useEvdStore(useCallback(state=>({
      type:state.data.thingTypes[props.uuid]
  })
    ,[props.uuid]))





return (
  <>

  <Divider orientation="left" style={{color:'white',borderTopColor:'rgba(255,255,255,0.12)',lineHeight: '1.5715px',paddingTop:'20px',paddingBottom:'5px'}}>
  Input:
  </Divider>
  <br/>
  //name
  <br/>
  <br/>
  <div style={{ color : 'white',display:'flex',justifyContent: 'space-between',alignItems:'center'}}>
  Quantity :
  <Input style= {{width :'20%'}} value = {props.inputQuantity}/>
  </div>
  <br/>
  <div style={{ color : 'white',display:'flex',justifyContent: 'space-between',alignItems:'center'}}>
  Region:
  <Button style= {{width :'20%'}} >
  Edit
  </Button>


  </div>




  <Divider orientation="left" style={{color:'white',borderTopColor:'rgba(255,255,255,0.12)',lineHeight: '1.5715px',paddingTop:'20px',paddingBottom:'5px'}}>
  Output:
  </Divider>
  <br/>
  <b style ={{color: 'white'}}> region.name </b>
  <br/>
  <br/>
  <div style={{ color : 'white',display:'flex',justifyContent: 'space-between',alignItems:'center'}}>
  Quantity :
  <Input style= {{width :'20%'}} value = {props.outputQuantity}/>
  </div>
  <br/>
  <div style={{ color : 'white',display:'flex',justifyContent: 'space-between',alignItems:'center'}}>
  Region:
  <Button style= {{width :'20%'}} >
  Edit
  </Button>

  </div>
  <Divider/>
  </>





)
}
