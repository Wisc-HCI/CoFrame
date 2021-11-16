import React from 'react';
import { List, Row, Button } from 'antd';
import {InputOutputComponent} from './InputOutputComponent'



export const MachineInOutTypeDetail = (props) => {
  let comKeys = null;
  if (props.input){
     comKeys = Object.keys(props.machine.inputs);
  }else {
     comKeys = Object.keys(props.machine.outputs);
  }
  
  return(
    <>
      <br/>
      <List 
        header={<Row align='middle' justify='space-between'>{props.input ? 'Inputs' : 'Outputs'}<Button>Add</Button></Row>}
        split={false}
        bordered 
        dataSource={comKeys} 
        renderItem = {(uuid) => (
          <List.Item> 
            <InputOutputComponent typeuuid={uuid} data = {props.machine[props.input ? 'inputs' : 'outputs'][uuid]}/>
          </List.Item>
        )}>

      </List>
    </>)


}
