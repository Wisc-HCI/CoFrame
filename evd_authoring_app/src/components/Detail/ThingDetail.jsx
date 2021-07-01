import React, {useCallback,useState} from 'react';



import { Empty } from 'antd';

import useEvdStore from '../../stores/EvdStore';


import { List, Space, Button, Popover,InputNumber,Divider,Col,Input } from 'antd';
import { DeleteOutlined, EllipsisOutlined,EditOutlined } from '@ant-design/icons';
import {eulerFromQuaternion, quaternionFromEuler} from './Geometry';

export const ThingDetail = ({uuid}) => {
    const RAD_2_DEG = 180 / Math.PI;
    const DEG_2_RAD = Math.PI / 180;

    const eulerVecToDegrees = (vec) => {
      return vec.map(v=>RAD_2_DEG*v)
    }
    const eulerVecToRadians = (vec) => {
      return vec.map(v=>DEG_2_RAD*v)
    }
    const [eulerValues, setEulerValues] = useState(eulerVecToDegrees(eulerFromQuaternion(uuid)));
    const DEGREE_STEP = uuid.step ? uuid.step*DEG_2_RAD : 1;
    let limits = [-360,360];
    let minMax = [-10,10];

    if (uuid.onlyPositive) {
      limits[0] = 0
    }

    const [inputVec, setInputVec] = useState(uuid);




    const {thing} = useEvdStore(useCallback(state=>({
        thing:state.data.thing[uuid],

    })
      ,[uuid]))

    const { TextArea } = Input;

    // const { deleteItem, setItemProperty } = useEvdStore(state=>({
    //     deleteItem:state.deleteItem,
    //     setItemProperty:state.setItemProperty
    // }));

    console.log(thing);

    return (
      <>
      <p>
      <b>Description:</b>
      </p>
      <Space/>

      <TextArea
        defaultValue={thing.description}
        disabled = {!thing.editable}
      />

      <Divider/>

  <Divider/>


</>

  )
}
