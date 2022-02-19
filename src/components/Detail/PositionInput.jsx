import React, {useState} from 'react';
//import { Space, Popover} from 'antd';
//import { EditOutlined } from '@ant-design/icons';
import {NumberInput} from '../NumberInput';
import {Box,Button,DropButton} from "grommet";
import {FormEdit} from "grommet-icons";

function PositionInput (props)  {
   const [inputVec, setInputVec] = useState(props.value);
   const [preVec, setPrevVec] = useState(null);
   let minMax = [-10,10];
   let steps = 0.01;
  
    if (props.value !== preVec){
      setPrevVec(props.value);
      setInputVec(props.value);
    }
   
   return(
     
     <div style={{ display:'flex',justifyContent: 'space-between',alignItems:'center'}}>
     <b style ={{color:'rgba(255, 255, 255, 0.85)'}}>Position:</b>
     
     <DropButton
      default
      icon={<FormEdit/>}
      dropAlign = {{right : 'left'}}
      dropContent={
       <Box>
         <h4 style={{ color: "red" }}>X</h4>
         <NumberInput
         min= {minMax[0]}
         max= {minMax[1]}
         value={inputVec[0]}
         onChange={(v)=>{if (typeof v === 'number') {
           let updatedVec = [v,inputVec[1],inputVec[2]];
           setInputVec(updatedVec)
           props.onChange(updatedVec)
         }}}
         />
         <h4 style={{ color: "lime" }}>Y</h4>
         <NumberInput
         min= {minMax[0]}
         max= {minMax[1]}
         value={inputVec[1]}
         onChange={(v)=>{if (typeof v === 'number') {
           let updatedVec = [inputVec[0],v,inputVec[2]]
           setInputVec(updatedVec)
           props.onChange(updatedVec)
         }}}
         />
         <h4 style={{ color: "blue" }}>Z</h4>
         <NumberInput
          min= {minMax[0]}
          max= {minMax[1]}
          value={inputVec[2]}
          onChange={(v)=>{if (typeof v === 'number') {
            let updatedVec = [inputVec[0],inputVec[1],v];
            setInputVec(updatedVec)
            props.onChange(updatedVec)
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
export default (PositionInput);
