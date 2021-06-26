import React, {useCallback,useState} from 'react';

import useEvdStore from '../../stores/EvdStore';


import { List, Space, Button, Popover,InputNumber,Divider,Col,Input } from 'antd';
import { DeleteOutlined, EllipsisOutlined,EditOutlined } from '@ant-design/icons';
import {eulerFromQuaternion, quaternionFromEuler} from './Geometry';
import {OrientationInput} from './OrientationInput';
import {PositionInput} from './PositionInput';

export const LocationDetail = ({uuid}) => {
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

    let oriMinMax = [-2*Math.PI,2*Math.PI]

    if (uuid.onlyPositive) {
      limits[0] = 0
    }






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
    


  <Divider/>


</>

  )
}
