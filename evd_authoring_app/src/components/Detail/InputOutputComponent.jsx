import React, {useCallback,useState} from 'react';
import useEvdStore from '../../stores/EvdStore';
import useGuiStore from '../../stores/GuiStore';
import { List, Space, Button, Popover,InputNumber,Divider,Col,Input,Drawer,Form,Row} from 'antd';

export const InputOutputComponent = (props) => {
  const {type} = useEvdStore(useCallback(state=>({
      type:state.data.thingTypes[props.typeuuid],

  })
    ,[props.typeuuid]))
    console.log(type.name);

return (
  <>
  <List >
  <List.Item>
  <List.Item.Meta style= {{padding: '0.1px'}} title = {type.name} description = {<Row style = {{padding: '-10px'}}justify={"space-between"}>
    <Col span = {12}><t>Quantity:</t> <Input style = {{width : "30%"}} compact value = {props.quantity}  /></Col>


    <Col span = {12}>
    <t style ={{paddingRight: '2px'}}>Region:</t>
    <Button style ={{width : "53%"}}>
      Edit
    </Button>
    </Col>
  </Row>}/>

  </List.Item>

  </List>
  </>
)
}
