import React, { useState } from 'react';
//import { Space, Popover} from 'antd';
//import { EditOutlined } from '@ant-design/icons';
import { NumberInput } from '../NumberInput';
import { Box, Button, DropButton, Text } from "grommet";
import { FormEdit } from "grommet-icons";

function PositionInput(props) {
  //  const [inputVec, setInputVec] = useState(props.value);
  //  const [preVec, setPrevVec] = useState(null);
  //  let minMax = [-10,10];
  //  let steps = 0.01;

  //   if (props.value !== preVec){
  //     setPrevVec(props.value);
  //     setInputVec(props.value);
  //   }

  return (

    <div style={{ display: 'flex', flexDirection: 'row', justifyContent: 'space-evenly', alignItems: 'center', paddingBottom: '3%' }}>
      <div>
        <Text style={{ color: 'rgba(255, 255, 255, 0.85)', paddingRight: "5%" }} >Position:</Text>
      </div>

      <div style={{ position: 'relative', left: '22.5%' }}>
        <DropButton
          primary
          icon={<FormEdit />}
          dropAlign={{ right: 'right', top: "bottom" }}
          dropProps={{ elevation: 'none',  round  : "xsmall"}}

          dropContent={
            <Box round = "xsmall" background = "grey" border = {{color : 'white', size : 'xsmall'}} width="xsmall" direction='column' elevation="none" pad="xsmall" justify='center'>
              <Box direction='row' elevation="none" pad="xsmall" justify='center' width="xsmall">
              <Text weight = "bolder" style={{ color: "red" , paddingRight : "7%"}}>X</Text>
              <NumberInput
              //  min= {minMax[0]}
              //  max= {minMax[1]}
              //  value={inputVec[0]}
              //  onChange={(v)=>{if (typeof v === 'number') {
              //    let updatedVec = [v,inputVec[1],inputVec[2]];
              //    setInputVec(updatedVec)
              //    props.onChange(updatedVec)
              //  }}}

              />
              </Box>

              <Box direction='row' elevation="none" pad="xsmall" justify='center' width="xsmall">
              <Text weight = "bolder" style={{ color: "lime" , paddingRight : "7%"}}>Y</Text>
              <NumberInput
              //  min= {minMax[0]}
              //  max= {minMax[1]}
              //  value={inputVec[1]}
              //  onChange={(v)=>{if (typeof v === 'number') {
              //    let updatedVec = [inputVec[0],v,inputVec[2]]
              //    setInputVec(updatedVec)
              //    props.onChange(updatedVec)
              //  }}}
              />
              </Box>

              <Box direction='row' elevation="none" pad="xsmall" justify='center' width="xsmall">
              <Text weight = "bolder" style={{ color: "blue" , paddingRight : "7%"}}>Z</Text>
              <NumberInput
              // min= {minMax[0]}
              // max= {minMax[1]}
              // value={inputVec[2]}
              // onChange={(v)=>{if (typeof v === 'number') {
              //   let updatedVec = [inputVec[0],inputVec[1],v];
              //   setInputVec(updatedVec)
              //   props.onChange(updatedVec)
              // }}}
              />
              </Box>

            </Box>
          }
        />
      </div>
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
