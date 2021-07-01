import React, {useCallback,useState} from 'react';



import { Empty } from 'antd';

import useEvdStore from '../../stores/EvdStore';


import { List, Space, Button, Popover,InputNumber,Divider,Col,Input } from 'antd';
import { DeleteOutlined, EllipsisOutlined,EditOutlined } from '@ant-design/icons';
import {eulerFromQuaternion, quaternionFromEuler} from './Geometry';

export const MachineDetail = ({uuid}) => {

    const {machine} = useEvdStore(useCallback(state=>({
        machine:state.data.machines[uuid],

    })
      ,[uuid]))

    const { TextArea } = Input;

    // const { deleteItem, setItemProperty } = useEvdStore(state=>({
    //     deleteItem:state.deleteItem,
    //     setItemProperty:state.setItemProperty
    // }));

    console.log(machine);

    return (
      <>
      <p>
      <b>Description:</b>
      </p>
      <Space/>

      <TextArea
        defaultValue={machine.description}
        disabled = {!machine.editable}
      />

      <Divider/>

    


  <Divider/>


</>

  )
}
