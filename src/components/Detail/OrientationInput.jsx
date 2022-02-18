import React, {useState} from 'react';
//import { Space, Button, Popover,InputNumber } from 'antd';
//import { EditOutlined } from '@ant-design/icons';
import {NumberInput} from '../NumberInput';
import {Box,Button,DropButton,Text} from "grommet";
import {FormEdit} from "grommet-icons";
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

 

const [preVec, setPrevVec] = useState(null);
if (props.value !== preVec){
  setPrevVec(props.value);
  setEulerValues(props.value);
}

  return(
    <div style={{ display:'flex',justifyContent: 'space-between',alignItems:'center'}}>
    <b style ={{color:'rgba(255, 255, 255, 0.85)'}}>Orientation:</b>
    <DropButton
    default
    icon={<FormEdit/>}
    dropAlign = {{right : 'left'}}
    dropContent={
      <Box>
        <h4 style={{ color: "red" }}>R</h4>
        <NumberInput
         min={limits[0]}
         max={limits[1]}
         value={eulerValues[0]}
         onChange={(v)=>{if (typeof v === 'number') {
           let updatedEulerValues = [v,eulerValues[1],eulerValues[2]];
           setEulerValues(updatedEulerValues);
           props.onChange(quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
         }}}
        />
        <h4 style={{ color: "lime" }}>P</h4>
        <NumberInput
         min={limits[0]}
         max={limits[1]}
         value={eulerValues[1]}
         onChange={(v)=>{if (typeof v === 'number') {
           let updatedEulerValues = [eulerValues[0],v,eulerValues[2]];
           setEulerValues(updatedEulerValues);
           props.onChange(quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
         }}}
        />
        <h4 style={{ color: "blue" }}>Y</h4>
        <NumberInput
         min={limits[0]}
         max={limits[1]}
         value={eulerValues[2]}
         onChange={(v)=>{if (typeof v === 'number') {
           let updatedEulerValues = [eulerValues[0],eulerValues[1],v];
           setEulerValues(updatedEulerValues);
           props.onChange(quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
         }}}
        />
        <Button primary onClick = {props.onClose}>Close</Button>
      </Box>
    }
   
    
  />  
    {/* <Button
      default
      icon={<FormEdit/>}
      style={{ margin: 3,width:"30%",height:"4%",placement:"right"}}
      onClick = {props.openStatus ? props.onClose : props.onOpen}
    >
    </Button> */}
    
  


  </div>

)

}
export default (OrientationInput);
