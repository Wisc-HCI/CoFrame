import React, { forwardRef } from "react";
import useStore from "../../stores/Store";
import { FiEdit2, FiSave } from "react-icons/fi";
import {
  IconButton,
  InputAdornment,
  OutlinedInput,
  InputLabel,
  FormControl,
} from "@mui/material";
import { IMaskInput } from "react-imask";
import { round } from "number-precision";

const Vector3MaskInput = forwardRef((props, ref) => {
  const { onChange, ...other } = props;
  return (
    <IMaskInput
      {...other}
      mask="XX , YY , ZZ"
      // overwrite
      // lazy
      blocks={{
        XX: { mask: Number, radix: ".", scale: 3, signed: true },
        YY: { mask: Number, radix: ".", scale: 3, signed: true },
        ZZ: { mask: Number, radix: ".", scale: 3, signed: true },
      }}
      inputRef={ref}
      onAccept={(value) => {
        const values = value.split(",").map(Number);
        onChange({
          target: {
            name: props.name,
            value: { x: values[0], y: values[1], z: values[2] },
          },
        });
      }}
    />
  );
});

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
      <InputLabel htmlFor="outlined-position-vector" color="primaryColor">
        Position
      </InputLabel>
      <OutlinedInput
        id="outlined-position-vector"
        label="Position"
        color="primaryColor"
        disabled={props.disabled || props.mode === 'rotate'}
        value={`${round(props.position.x,3)} , ${round(props.position.y,3)} , ${round(props.position.z,3)}`}
        inputComponent={Vector3MaskInput}
        onChange={(e) =>
          updateItemSimpleProperty(props.itemID, "position", e.target.value)
        }
        endAdornment={
          <InputAdornment position="end">
            <IconButton
              disabled={props.disabled || props.mode === 'rotate'}
              color={props.mode === 'translate' ? 'primaryColor' : 'inherit'}
              aria-label="toggle password visibility"
              onClick={props.mode === 'translate' ? handleClose : buttonClick}
              // onMouseDown={open ? handleClose : buttonClick}
            >
              {props.mode === 'translate' ? <FiSave/> : <FiEdit2/>}
            </IconButton>
          </InputAdornment>
        }
      />
    </FormControl>
  );
}
export default PositionInput;
