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


const CompoundInput = forwardRef(
  ({ onChange, value, disabled, ...other }, ref) => {
    return (
      <Stack direction="row" spacing={1} ref={ref} divider={<Divider orientation="vertical" flexItem />}>
        <Input
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
export default PositionInput;
