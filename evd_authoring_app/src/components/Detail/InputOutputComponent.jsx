import React, { useCallback } from 'react';
import useEvdStore from '../../stores/EvdStore';
import useGuiStore from '../../stores/GuiStore';
import { List, Space, Button, InputNumber, Row } from 'antd';

export const InputOutputComponent = (props) => {
  const type = useEvdStore(useCallback(state => state.data.thingTypes[props.typeuuid], [props.typeuuid]));
  console.log(type)

  const setChildrenDrawer = useGuiStore(state => state.setChildrenDrawer)



  const {setSecondaryFocusItem} = useGuiStore(state => ({
    setSecondaryFocusItem : state.setSecondaryFocusItem
  }));



  return (
    <List.Item.Meta title={type.name} description={
      <List
          size='small'
          split={false}
          dataSource={props.data}
          renderItem={(data)=>(
            <List.Item style={{padding:0}}>
              <Row justify='space-between' style={{width:'100%', padding:10, marginTop: 4, borderRadius: 4, backgroundColor:'rgba(100,100,100,0.2)'}}>
                <Space>
                  <t>Quantity:</t>
                  <InputNumber min = {1} style={{ maxWidth: 50 }} compact defaultValue={data.quantity} />
                </Space>
                <Space>
                  <t style={{ paddingRight: '4px' }}>Region:</t>
                  <Button onClick={()=>{setChildrenDrawer(true); setSecondaryFocusItem('region',data.region_uuid)}}>
                    Edit
                  </Button>
                </Space>
              </Row>
            </List.Item>
          )}
      />
  }/>)
}
