import React, {useState} from 'react';


import { Space, Button, Popover,InputNumber } from 'antd';
import { EditOutlined } from '@ant-design/icons';
import {eulerFromQuaternion, quaternionFromEuler} from './Geometry';


const RAD_2_DEG = 180 / Math.PI;
const DEG_2_RAD = Math.PI / 180;

const eulerVecToDegrees = (vec) => {
  return vec.map(v=>RAD_2_DEG*v)
}
const eulerVecToRadians = (vec) => {
  return vec.map(v=>DEG_2_RAD*v)
}

function OrientationInput (props)  {
 const [eulerValues, setEulerValues] = useState(eulerVecToDegrees(eulerFromQuaternion(props.value)));

 let limits = [-360,360];
 let steps = Math.PI / 12;
 if (props.onlyPositive) {
   limits[0] = 0
 }

  return(
    <div style={{ display:'flex',justifyContent: 'space-between',alignItems:'center'}}>
    <b style ={{color:'rgba(255, 255, 255, 0.85)'}}>Orientation:</b>
    <Popover
    placement="left"
    content={
      <Space>
        <h4 style={{ color: "red" }}>R</h4>
        <InputNumber
         min={limits[0]}
         max={limits[1]}
         step={steps}
         precision={2}
         style={{ margin: '0 16px' }}
         defaultValue={eulerValues[0]}
         onChange={(v)=>{if (typeof v === 'number') {
           let updatedEulerValues = [v,eulerValues[1],eulerValues[2]];
           setEulerValues(updatedEulerValues);
           props.onChange(quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
         }}}
        />
        <h4 style={{ color: "lime" }}>P</h4>
        <InputNumber
         min={limits[0]}
         max={limits[1]}
         step={steps}
         precision={2}
         style={{ margin: '0 16px' }}
         defaultValue={eulerValues[1]}
         onChange={(v)=>{if (typeof v === 'number') {
           let updatedEulerValues = [eulerValues[0],v,eulerValues[2]];
           setEulerValues(updatedEulerValues);
           props.onChange(quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
         }}}
        />
        <h4 style={{ color: "blue" }}>Y</h4>
        <InputNumber
         min={limits[0]}
         max={limits[1]}
         step={steps}
         precision={2}
         style={{ margin: '0 16px' }}
         defaultValue={eulerValues[2]}
         onChange={(v)=>{if (typeof v === 'number') {
           let updatedEulerValues = [eulerValues[0],eulerValues[1],v];
           setEulerValues(updatedEulerValues);
           props.onChange(quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
         }}}
        />
      </Space>
    }
    title="Set Orientation"
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

)

}
export default (OrientationInput);
