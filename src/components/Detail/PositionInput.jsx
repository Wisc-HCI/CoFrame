import React, { forwardRef } from "react";
import useStore from "../../stores/Store";
import { FiEdit2, FiSave } from "react-icons/fi";
import {
  IconButton,
  InputAdornment,
  OutlinedInput,
  InputLabel,
  FormControl,
  Input,
  Divider,
  Stack,
} from "@mui/material";
import { strip } from "number-precision";

// const strip = (v)=>{
//   console.log(v)
//   return Number(Number(v).toFixed(3))
// }

const CompoundInput = forwardRef(
  ({ onChange, value, disabled, ...other }, ref) => {
    return (
      <Stack direction="row" spacing={1} ref={ref} divider={<Divider orientation="vertical" flexItem />}>
        <Input
          size="small"
          disabled={disabled}
          disableUnderline
          className={other.className}
          label={null}
          value={value[0]}
          inputProps={{step:0.1}}
          onFocus={other.onFocus}
          onBlur={other.onBlur}
          onChange={(e)=>{onChange({target:{name:other.name,value:{x:strip(e.target.value),y:value[1],z:value[2]}}})}}
          type="number"
          margin="dense"
        />
        <Input
          size="small"
          disabled={disabled}
          disableUnderline
          className={other.className}
          label={null}
          value={value[1]}
          inputProps={{step:0.1}}
          onFocus={other.onFocus}
          onBlur={other.onBlur}
          onChange={(e)=>{onChange({target:{name:other.name,value:{x:value[0],y:strip(e.target.value),z:value[2]}}})}}
          type="number"
          margin="dense"
        />
        <Input
          size="small"
          disabled={disabled}
          disableUnderline
          className={other.className}
          label={null}
          value={value[2]}
          inputProps={{step:0.1}}
          onFocus={other.onFocus}
          onBlur={other.onBlur}
          onChange={(e)=>{onChange({target:{name:other.name,value:{x:value[0],y:value[1],z:strip(e.target.value)}}})}}
          type="number"
          margin="dense"
        />
      </Stack>
    );
  }
);

// function PositionInput({ itemID, disabled, mode }) {
//   return <Card variant='outline'></Card>;
// }

function PositionInput(props) {
  const updateItemSimpleProperty = useStore(
    (state) => state.updateItemSimpleProperty
  );
  const addFocusItem = useStore((state) => state.addFocusItem);

  function buttonClick() {
    addFocusItem("translate", true);
  }

  function handleClose() {
    addFocusItem(props.itemID, true);
  }

  return (
    <FormControl>
      <InputLabel htmlFor="outlined-position-vector" color="primaryColor" shrink>
        Position
      </InputLabel>
      <OutlinedInput
        notched
        size="small"
        id="outlined-position-vector"
        label="Position"
        color="primaryColor"
        disabled={props.disabled || props.mode === "rotate"}
        value={[props.position.x, props.position.y, props.position.z]}
        inputComponent={CompoundInput}
        onChange={(e) =>
          updateItemSimpleProperty(props.itemID, "position", e.target.value)
        }
        endAdornment={
          <InputAdornment position="end">
            <IconButton
              size="small"
              disabled={props.disabled || props.mode === "rotate"}
              color={props.mode === "translate" ? "primaryColor" : "inherit"}
              aria-label="toggle password visibility"
              onClick={props.mode === "translate" ? handleClose : buttonClick}
              // onMouseDown={open ? handleClose : buttonClick}
            >
              {props.mode === "translate" ? <FiSave /> : <FiEdit2 />}
            </IconButton>
          </InputAdornment>
        }
      />
    </FormControl>
  );
}

export function SimplePositionInput({
  value={x:0,y:0,z:0},
  onChange=(value)=>{},
  disabled=false,
  active=false,
  onToggleActivity=(newValue)=>{}
}) {

  return (
    <FormControl>
      <InputLabel htmlFor="outlined-position-vector" color="primaryColor" shrink>
        Position
      </InputLabel>
      <OutlinedInput
        notched
        size="small"
        id="outlined-position-vector"
        label="Position"
        color="primaryColor"
        disabled={disabled || !active}
        value={[value.x, value.y, value.z]}
        inputComponent={CompoundInput}
        onChange={onChange}
        endAdornment={
          <InputAdornment position="end">
            <IconButton
              size="small"
              disabled={disabled}
              color="primary"
              aria-label="toggle editing position"
              onClick={()=>onToggleActivity(!active)}
              // onMouseDown={open ? handleClose : buttonClick}
            >
              {active ? <FiSave /> : <FiEdit2 />}
            </IconButton>
          </InputAdornment>
        }
      />
    </FormControl>
  );
}

export default PositionInput;
