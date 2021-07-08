import React, {useState} from 'react';
import { Space, Button, Popover,InputNumber } from 'antd';
import { EditOutlined } from '@ant-design/icons';

function PositionInput (props)  {
   const [inputVec, setInputVec] = useState(props.value);
   let minMax = [-10,10];
   let steps = 0.01;

   return(
     <div style={{ display:'flex',justifyContent: 'space-between',alignItems:'center'}}>
     <b style ={{color:'rgba(255, 255, 255, 0.85)'}}>Position:</b>
     <Popover
     placement="left"
     content={
       <Space>
         <h4 style={{ color: "red" }}>X</h4>
         <InputNumber
         min= {minMax[0]}
         max= {minMax[1]}
         step= {steps}
         precision={2}
         style={{ margin: '0 16px' }}
         defaultValue={inputVec[0]}
         onChange={(v)=>{if (typeof v === 'number') {
           let updatedVec = [v,inputVec[1],inputVec[2]];
           setInputVec(updatedVec)
           props.onChange(updatedVec)
         }}}
         />
         <h4 style={{ color: "lime" }}>Y</h4>
         <InputNumber
         min= {minMax[0]}
         max= {minMax[1]}
         step= {steps}
         precision={2}
         style={{ margin: '0 16px' }}
         defaultValue={inputVec[1]}
         onChange={(v)=>{if (typeof v === 'number') {
           let updatedVec = [inputVec[0],v,inputVec[2]]
           setInputVec(updatedVec)
           props.onChange(updatedVec)
         }}}
         />
         <h4 style={{ color: "blue" }}>Z</h4>
         <InputNumber
          min= {minMax[0]}
          max= {minMax[1]}
          step={steps}
          precision={2}
          style={{ margin: '0 16px' }}
          defaultValue={inputVec[2]}
          onChange={(v)=>{if (typeof v === 'number') {
            let updatedVec = [inputVec[0],inputVec[1],v];
            setInputVec(updatedVec)
            props.onChange(updatedVec)
          }}}
         />
       </Space>
     }
     title="Set Position"
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
export default (PositionInput);
