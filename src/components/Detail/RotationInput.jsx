import React, { useState } from 'react';
import { NumberInput } from '../Elements/NumberInput';
import { Box, DropButton, Text,Button } from "grommet";
import { FormEdit } from "grommet-icons";
import { eulerFromQuaternion, quaternionFromEuler } from './Geometry';
import useStore from '../../stores/Store';
import {FiX} from 'react-icons/fi';


//RPY is Euler 
//XYZW is Quaternion

function RotationInput(props) {
  const RAD_2_DEG = 180 / Math.PI;
  const DEG_2_RAD = Math.PI / 180;
  const rotValue = [props.rotation.w, props.rotation.x, props.rotation.y, props.rotation.z];
  const updateItemRotationProperty = useStore(state => state.updateItemRotationProperty);
  const eulerVecToDegrees = (vec) => {
    return vec.map(v => RAD_2_DEG * v)
  }
  const eulerVecToRadians = (vec) => {
    return vec.map(v => DEG_2_RAD * v)
  }
  const [eulerValues, setEulerValues] = useState(eulerVecToDegrees(eulerFromQuaternion(rotValue)));
  let limits = [-360, 360];
  const addFocusItem = useStore(state => state.addFocusItem);
  const [open,setOpen] = useState(false);

  function buttonClick(){
    addFocusItem("rotate", true);
    setOpen(!open);
  }

  function handleClose(){
    addFocusItem(props.itemID, true);
    setOpen(!open);
  }

  return (
    <Box round="xsmall"
      background="black"
      pad="small"
      width="100%">
      <div style={{ display: 'flex', flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center' }}>
        <Text style={{ color: 'rgba(255, 255, 255, 0.85)' }}>Rotation:</Text>
        <DropButton
          primary
          label="Edit"
          icon={<FormEdit />}
          dropAlign={{ right: 'right', top: "bottom" }}
          dropProps={{ elevation: 'none', round: "xsmall" }}
          onOpen={buttonClick}
          open = {open}
          dropContent={
            <Box round="xsmall" background="grey" border={{ color: 'white', size: 'xsmall' }} width="small" direction='column'
              elevation="none" pad="xsmall" justify='center'>
              <Box direction='row' elevation="none" pad="xsmall" justify='center' width="small">
                <Text weight="bolder" style={{ color: "red", paddingRight: "7%" }}>R</Text>
                <NumberInput
                  min={limits[0]}
                  max={limits[1]}
                  value={eulerValues[0]}

                  onChange={(v) => {
                    if (typeof v === 'number') {
                      let updatedEulerValues = [v, eulerValues[1], eulerValues[2]];
                      setEulerValues(updatedEulerValues);
                      updateItemRotationProperty(props.itemID, quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
                    }
                  }}
                />
              </Box>

              <Box direction='row' elevation="none" pad="xsmall" justify='center' width="small">
                <Text weight="bolder" style={{ color: "lime", paddingRight: "7%" }}>P</Text>
                <NumberInput
                  min={limits[0]}
                  max={limits[1]}
                  value={eulerValues[1]}
                  onChange={(v) => {
                    if (typeof v === 'number') {
                      let updatedEulerValues = [eulerValues[0], v, eulerValues[2]];
                      setEulerValues(updatedEulerValues);
                      updateItemRotationProperty(props.itemID, quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
                    }
                  }}
                />
              </Box>

              <Box direction='row' elevation="none" pad="xsmall" justify='center' width="small">
                <Text weight="bolder" style={{ color: "blue", paddingRight: "7%" }}>Y</Text>
                <NumberInput
                  min={limits[0]}
                  max={limits[1]}
                  value={eulerValues[2]}
                  onChange={(v) => {
                    if (typeof v === 'number') {
                      let updatedEulerValues = [eulerValues[0], eulerValues[1], v];
                      setEulerValues(updatedEulerValues);
                      updateItemRotationProperty(props.itemID, quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
                    }
                  }}
                />
              </Box>
              <Button onClick = {handleClose} size = "small" margin = "xxsmall" primary icon={<FiX />} label="Close" color="#ab4646" />
            </Box>
          }
        />
      </div>
    </Box>

  )

}
export default (RotationInput);
