import React, {useCallback,useState} from 'react';



import { Empty } from 'antd';

import useEvdStore from '../../stores/EvdStore';
import useGuiStore from '../../stores/GuiStore';
import { List, Space, Button, Popover,InputNumber,Divider,Col,Input,Drawer,Form } from 'antd';
import {InputOutputComponent} from './InputOutputComponent'



export const MachineInOutTypeDetail = (props) => {
  let comKeys = null;
  if (props.input === "true"){
     comKeys = Object.keys(props.machine.inputs).map((item) => item);
  }else {
     comKeys = Object.keys(props.machine.outputs).map((item) => item);
  }





  let inputComponent = null;
  if (comKeys) {
    inputComponent = comKeys.map((item) => {
         return <InputOutputComponent typeuuid = {item} quantity = {props.machine.inputs[item][0].quantity}/>
    } )
  }else{
    inputComponent = <b> No Input exist in this machine </b>
  }


  console.log(inputComponent)
  return(
    <>
    <br/>
    <List bordered dataSource={inputComponent} renderItem = {component => (<List.Item> {component}</List.Item>)}>

    </List>


    </>)


}
