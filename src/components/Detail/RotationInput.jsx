import React, { forwardRef, useCallback } from "react";
import useStore from "../../stores/Store";
import { FiEdit2, FiSave } from "react-icons/fi";
import {
  eulerFromQuaternion,
  quaternionFromEuler,
  quaternionVecToObject,
} from "../../helpers/geometry";
import {
  IconButton,
  InputAdornment,
  OutlinedInput,
  InputLabel,
  FormControl,
  Stack,
  Divider,
  Input
} from "@mui/material";
// import { IMaskInput } from "react-imask";
import shallow from "zustand/shallow";
import { strip } from "number-precision";

const CompoundInput = forwardRef(
  ({ onChange, value, disabled, ...other }, ref) => {
    return (
      <Stack
        direction="row"
        spacing={1}
        divider={<Divider orientation="vertical" flexItem />}
      >
        <Input
          autoFocus
          disabled={disabled}
          disableUnderline
          className={other.className}
          label={null}
          value={strip(value[0])}
          inputProps={{ step: 1 }}
          onFocus={other.onFocus}
          onBlur={other.onBlur}
          onChange={(e) => {
            const newVec = [strip(e.target.value), value[1], value[2]];
            const quaternionObj = quaternionVecToObject(
              quaternionFromEuler(eulerVecToRadians(newVec))
            )
            onChange({
              target: {
                name: other.name,
                value: quaternionObj,
              },
            });
          }}
          type="number"
          margin="dense"
        />
        <Input
          disabled={disabled}
          disableUnderline
          className={other.className}
          label={null}
          value={strip(value[1])}
          inputProps={{ step: 1 }}
          onFocus={other.onFocus}
          onBlur={other.onBlur}
          onChange={(e) => {
            const newVec = [value[0], strip(e.target.value), value[2]];
            const quaternionObj = quaternionVecToObject(
              quaternionFromEuler(eulerVecToRadians(newVec))
            )
            onChange({
              target: {
                name: other.name,
                value: quaternionObj,
              },
            });
          }}
          type="number"
          margin="dense"
        />
        <Input
          disabled={disabled}
          disableUnderline
          className={other.className}
          label={null}
          value={strip(value[2])}
          inputProps={{ step: 1 }}
          onFocus={other.onFocus}
          onBlur={other.onBlur}
          onChange={(e) => {
            const newVec = [value[0], value[1], strip(e.target.value)];
            const quaternionObj = quaternionVecToObject(
              quaternionFromEuler(eulerVecToRadians(newVec))
            )
            onChange({
              target: {
                name: other.name,
                value: quaternionObj,
              },
            });
          }}
          type="number"
          margin="dense"
        />
      </Stack>
    );
  }
);



//RPY is Euler
//XYZW is Quaternion
const RAD_2_DEG = 180 / Math.PI;
const DEG_2_RAD = Math.PI / 180;

const eulerVecToDegrees = (vec) => {
  return vec.map((v) => RAD_2_DEG * v);
};
const eulerVecToRadians = (vec) => {
  return vec.map((v) => DEG_2_RAD * v);
};

function RotationInput(props) {
  const quatObj = props.rotation;
  const quatVec = [quatObj.w, quatObj.x, quatObj.y, quatObj.z];
  const euler = eulerVecToDegrees(eulerFromQuaternion(quatVec));
  // const euler = eulerVecToDegrees(eulerFromQuaternion(quat));
  // console.log('rerendering with valuestring:',valueString)

  const updateItemSimpleProperty = useStore(
    (state) => state.updateItemSimpleProperty
  );
  const addFocusItem = useStore((state) => state.addFocusItem);

  function buttonClick() {
    addFocusItem("rotate", true);
  }

  function handleClose() {
    addFocusItem(props.itemID, true);
  }

  // console.log('EULER',{euler,string:`${euler[0]} , ${euler[1]} , ${euler[2]}`})

  return (
    <FormControl >
      <InputLabel htmlFor="outlined-rotation-vector" color="primaryColor" shrink>
        Rotation
      </InputLabel>
      <OutlinedInput
        notched
        id="outlined-rotation-vector"
        label="Position"
        color="primaryColor"
        disabled={props.disabled || props.mode === "translate"}
        value={euler}
        inputComponent={CompoundInput}
        onChange={(e) =>
          updateItemSimpleProperty(props.itemID, "rotation", e.target.value)
        }
        inputProps={{shrink:true}}
        margin='dense'
        endAdornment={
          <InputAdornment position="end">
            <IconButton
              disabled={props.disabled || props.mode === "translate"}
              color={props.mode === "rotate" ? "primaryColor" : "inherit"}
              onClick={props.mode === "rotate" ? handleClose : buttonClick}
              // onMouseDown={open ? handleClose : buttonClick}
            >
              {props.mode === "rotate" ? <FiSave /> : <FiEdit2 />}
            </IconButton>
          </InputAdornment>
        }
      />
    </FormControl>
  );
}
export default RotationInput;

// function RotationInput(props) {
//   const RAD_2_DEG = 180 / Math.PI;
//   const DEG_2_RAD = Math.PI / 180;
//   const rotValue = [props.rotation.w, props.rotation.x, props.rotation.y, props.rotation.z];
//   const updateItemRotationProperty = useStore(state => state.updateItemRotationProperty);
//   const eulerVecToDegrees = (vec) => {
//     return vec.map(v => RAD_2_DEG * v)
//   }
//   const eulerVecToRadians = (vec) => {
//     return vec.map(v => DEG_2_RAD * v)
//   }
//   const [eulerValues, setEulerValues] = useState(eulerVecToDegrees(eulerFromQuaternion(rotValue)));
//   let limits = [-360, 360];
//   const addFocusItem = useStore(state => state.addFocusItem);
//   const [open,setOpen] = useState(false);

//   function buttonClick(){
//     addFocusItem("rotate", true);
//     setOpen(!open);
//   }

//   function handleClose(){
//     addFocusItem(props.itemID, true);
//     setOpen(!open);
//   }

//   return (
//     <Box round="xsmall"
//       background="black"
//       pad="small"
//       width="100%">
//       <div style={{ display: 'flex', flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center' }}>
//         <Text style={{ color: 'rgba(255, 255, 255, 0.85)' }}>Rotation:</Text>
//         <DropButton
//           primary
//           label="Edit"
//           icon={<FormEdit />}
//           dropAlign={{ right: 'right', top: "bottom" }}
//           dropProps={{ elevation: 'none', round: "xsmall" }}
//           onOpen={buttonClick}
//           open = {open}
//           dropContent={
//             <Box round="xsmall" background="grey" border={{ color: 'white', size: 'xsmall' }} width="small" direction='column'
//               elevation="none" pad="xsmall" justify='center'>
//               <Box direction='row' elevation="none" pad="xsmall" justify='center' width="small">
//                 <Text weight="bolder" style={{ color: "red", paddingRight: "7%" }}>R</Text>
//                 <NumberInput
//                   min={limits[0]}
//                   max={limits[1]}
//                   value={eulerValues[0]}

//                   onChange={(v) => {
//                     if (typeof v === 'number') {
//                       let updatedEulerValues = [v, eulerValues[1], eulerValues[2]];
//                       setEulerValues(updatedEulerValues);
//                       updateItemRotationProperty(props.itemID, quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
//                     }
//                   }}
//                 />
//               </Box>

//               <Box direction='row' elevation="none" pad="xsmall" justify='center' width="small">
//                 <Text weight="bolder" style={{ color: "lime", paddingRight: "7%" }}>P</Text>
//                 <NumberInput
//                   min={limits[0]}
//                   max={limits[1]}
//                   value={eulerValues[1]}
//                   onChange={(v) => {
//                     if (typeof v === 'number') {
//                       let updatedEulerValues = [eulerValues[0], v, eulerValues[2]];
//                       setEulerValues(updatedEulerValues);
//                       updateItemRotationProperty(props.itemID, quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
//                     }
//                   }}
//                 />
//               </Box>

//               <Box direction='row' elevation="none" pad="xsmall" justify='center' width="small">
//                 <Text weight="bolder" style={{ color: "blue", paddingRight: "7%" }}>Y</Text>
//                 <NumberInput
//                   min={limits[0]}
//                   max={limits[1]}
//                   value={eulerValues[2]}
//                   onChange={(v) => {
//                     if (typeof v === 'number') {
//                       let updatedEulerValues = [eulerValues[0], eulerValues[1], v];
//                       setEulerValues(updatedEulerValues);
//                       updateItemRotationProperty(props.itemID, quaternionFromEuler(eulerVecToRadians(updatedEulerValues)));
//                     }
//                   }}
//                 />
//               </Box>
//               <Button onClick = {handleClose} size = "small" margin = "xxsmall" primary icon={<FiX />} label="Close" color="#ab4646" />
//             </Box>
//           }
//         />
//       </div>
//     </Box>

//   )

// }
// export default (RotationInput);
