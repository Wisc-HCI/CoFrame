import React, {useCallback} from 'react';



import { Empty } from 'antd';

import useEvdStore from '../../stores/EvdStore';


import { List, Space, Button, Popover,InputNumber,Divider,Col,Input } from 'antd';
import { DeleteOutlined, EllipsisOutlined,EditOutlined } from '@ant-design/icons';





export const LocationDetail = ({uuid}) => {

    const {location} = useEvdStore(useCallback(state=>({
        location:state.data.locations[uuid],

    })
      ,[uuid]))

    const { TextArea } = Input;

    // const { deleteItem, setItemProperty } = useEvdStore(state=>({
    //     deleteItem:state.deleteItem,
    //     setItemProperty:state.setItemProperty
    // }));

    console.log(location);

    return (
      <>
      <p>
      <b>Description:</b>
      </p>
      <Space/>

      <TextArea
        defaultValue={location.description}
        disabled = {!location.editable}
      />

      <Divider/>

      <div style={{ display:'flex',justifyContent: 'space-between',alignItems:'center'}}>
      <b>Position:</b>
      <Popover
      placement="left"
      content={
        <Space>
          <h4 style={{ color: "red" }}>R</h4>
          <InputNumber
            precision={2}
            style={{ margin: "0 16px" }}
            onChange={(v) => {
              if (typeof v === "number") {
              }
            }}
          />
          <h4 style={{ color: "lime" }}>P</h4>
          <InputNumber
            precision={2}
            style={{ margin: "0 16px" }}
            onChange={(v) => {
              if (typeof v === "number") {
              }
            }}
          />
          <h4 style={{ color: "blue" }}>Y</h4>
          <InputNumber
            precision={2}
            style={{ margin: "0 16px" }}
            onChange={(v) => {
              if (typeof v === "number") {
              }
            }}
          />
        </Space>
      }
      title="Set Rotation"
      trigger="click"

    >
      <Button
        block
        icon={<EditOutlined />}
        style={{ margin: 3,width:"30%",height:"4%",placement:"right"}}
      >
      </Button>
    </Popover>
    </div>

    <br />
    <div style={{ display:'flex',justifyContent: 'space-between',alignItems:'center'}}>
    <b>Orientation:</b>
    <Popover
    placement="left"
    content={
      <Space>
        <h4 style={{ color: "red" }}>R</h4>
        <InputNumber
          precision={2}
          style={{ margin: "0 16px" }}
          onChange={(v) => {
            if (typeof v === "number") {
            }
          }}
        />
        <h4 style={{ color: "lime" }}>P</h4>
        <InputNumber
          precision={2}
          style={{ margin: "0 16px" }}
          onChange={(v) => {
            if (typeof v === "number") {
            }
          }}
        />
        <h4 style={{ color: "blue" }}>Y</h4>
        <InputNumber
          precision={2}
          style={{ margin: "0 16px" }}
          onChange={(v) => {
            if (typeof v === "number") {
            }
          }}
        />
      </Space>
    }
    title="Set Rotation"
    trigger="click"

  >
    <Button
      block
      icon={<EditOutlined />}
      style={{ margin: 3,width:"30%",height:"4%",placement:"right"}}
    >
    </Button>
  </Popover>


  </div>









  <Divider/>


</>









  )
}
