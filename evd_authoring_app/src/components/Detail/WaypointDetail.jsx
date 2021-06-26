import React, {useCallback,useState} from 'react';



import { Empty } from 'antd';

import useEvdStore from '../../stores/EvdStore';


import { List, Space, Button, Popover,InputNumber,Divider,Col,Input } from 'antd';
import { DeleteOutlined, EllipsisOutlined,EditOutlined } from '@ant-design/icons';
import {eulerFromQuaternion, quaternionFromEuler} from './Geometry';

export const WaypointDetail = ({uuid}) => {
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




    const {waypoint} = useEvdStore(useCallback(state=>({
        waypoint:state.data.waypoint[uuid],

    })
      ,[uuid]))

    const { TextArea } = Input;

    // const { deleteItem, setItemProperty } = useEvdStore(state=>({
    //     deleteItem:state.deleteItem,
    //     setItemProperty:state.setItemProperty
    // }));

    console.log(waypoint);

    return (
      <>
      <p>
      <b>Description:</b>
      </p>
      <Space/>

      <TextArea
        defaultValue={waypoint.description}
        disabled = {!waypoint.editable}
      />

      <Divider/>

      <div style={{ display:'flex',justifyContent: 'space-between',alignItems:'center'}}>
      <b>Position:</b>
      <Popover
      placement="left"
      content={
        <Space>
          <h4 style={{ color: "red" }}>X</h4>
          <InputNumber
          min= {minMax[0]}
          max= {minMax[1]}
          step={uuid.step}
          precision={2}
          style={{ margin: '0 16px' }}
          defaultValue={inputVec[0]}
          onChange={(v)=>{if (typeof v === 'number') {
            let updatedVec = [v,inputVec[1],inputVec[2]];
            setInputVec(updatedVec)
            uuid.onChange(updatedVec)
          }}}
          />
          <h4 style={{ color: "lime" }}>Y</h4>
          <InputNumber
          min= {minMax[0]}
          max= {minMax[1]}
          step={uuid.step}
          precision={2}
          style={{ margin: '0 16px' }}
          defaultValue={inputVec[1]}
          onChange={(v)=>{if (typeof v === 'number') {
            let updatedVec = [inputVec[0],v,inputVec[2]]
            setInputVec(updatedVec)
            uuid.onChange(updatedVec)
          }}}
          />
          <h4 style={{ color: "blue" }}>Z</h4>
          <InputNumber
           min= {minMax[0]}
           max= {minMax[1]}
           step={uuid.step}
           precision={2}
           style={{ margin: '0 16px' }}
           defaultValue={inputVec[2]}
           onChange={(v)=>{if (typeof v === 'number') {
             let updatedVec = [inputVec[0],inputVec[1],v];
             setInputVec(updatedVec)
             uuid.onChange(updatedVec)
           }}}
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
         min={limits[0]}
         max={limits[1]}
         step={DEGREE_STEP}
         precision={2}
         style={{ margin: '0 16px' }}
         defaultValue={eulerValues[0]}
         onChange={(v)=>{if (typeof v === 'number') {
           let updatedEulerValues = [v,eulerValues[1],eulerValues[2]];
           setEulerValues(updatedEulerValues);
           uuid.onChange(quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
         }}}
        />
        <h4 style={{ color: "lime" }}>P</h4>
        <InputNumber
         min={limits[0]}
         max={limits[1]}
         step={DEGREE_STEP}
         precision={2}
         style={{ margin: '0 16px' }}
         defaultValue={eulerValues[1]}
         onChange={(v)=>{if (typeof v === 'number') {
           let updatedEulerValues = [eulerValues[0],v,eulerValues[2]];
           setEulerValues(updatedEulerValues);
           uuid.onChange(quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
         }}}
        />
        <h4 style={{ color: "blue" }}>Y</h4>
        <InputNumber
        min={limits[0]}
         max={limits[1]}
         step={DEGREE_STEP}
         precision={2}
         style={{ margin: '0 16px' }}
         defaultValue={eulerValues[2]}
         onChange={(v)=>{if (typeof v === 'number') {
           let updatedEulerValues = [eulerValues[0],eulerValues[1],v];
           setEulerValues(updatedEulerValues);
           uuid.onChange(quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
         }}}
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
