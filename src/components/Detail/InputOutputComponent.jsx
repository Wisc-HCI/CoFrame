import React, { useCallback } from 'react';
import useStore from '../../stores/Store';
//import { List, Space, Button, InputNumber, Row } from 'antd';
import {List, Button,Accordion} from "grommet";
import {NumberInput} from "../NumberInput";

export const InputOutputComponent = (props) => {
  //const type = useStore(useCallback(state => state.data.thingTypes[props.typeuuid], [props.typeuuid]));

  // const setChildrenDrawer = useStore(state => state.setChildrenDrawer)

  const type = {
    name : "This is the Name"
  }

  const data = {
    quantity : 4,
    region_uuid : "123"
  }

  // const {setSecondaryFocusItem} = useStore(state => ({
  //   setSecondaryFocusItem : state.setSecondaryFocusItem
  // }));



  return (
  //   <List.Item.Meta title={type.name} description={
  //     <List
  //         size='small'
  //         split={false}
  //         dataSource={props.data}
  //         renderItem={(data)=>(
  //           <List.Item style={{padding:0}}>
  //             <Row justify='space-between' style={{width:'100%', padding:10, marginTop: 4, borderRadius: 4, backgroundColor:'rgba(100,100,100,0.2)'}}>
  //               <Space>
  //                 <b>Quantity:</b>
  //                 <InputNumber min = {1} style={{ maxWidth: 50 }} defaultValue={data.quantity} />
  //               </Space>
  //               <Space>
  //                 <b style={{ paddingRight: '4px' }}>Region:</b>
  //                 <Button onClick={()=>{setSecondaryFocusItem('region',data.region_uuid)}}>
  //                   Edit
  //                 </Button>
  //               </Space>
  //             </Row>
  //           </List.Item>
  //         )}
  //     />
  // }/>
  <>
    <Accordion>

    </Accordion>
  </>

  
  )
}
