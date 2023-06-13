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
import { useNumeric, NUMERIC_STATUS } from "../Elements/useNumeric";
import { Spinner } from "../Elements/NumberInput";

// const strip = (v)=>{
//   console.log(v)
//   return Number(Number(v).toFixed(3))
// }

const CompoundInput = forwardRef(
  ({ onChange, value, disabled, ...other }, ref) => {
    const {
      textValue: textValue0,
      status: status0,
      onChange: onChangeInner0,
      onStepUp: onStepUp0,
      onStepDown: onStepDown0,
    } = useNumeric({
      initial: value[0],
      stepSize: other.step || 0.1,
      min: other.min == undefined ? -Infinity : other.min,
      max: other.max == undefined ? -Infinity : other.max,
      onValidChange: async (v) => {
        console.log('changing x');
        onChange({
          target: {
            name: other.name,
            value: { x: v, y: value[1], z: value[2] },
          },
        })
      },
    });

    const {
      textValue: textValue1,
      status: status1,
      onChange: onChangeInner1,
      onStepUp: onStepUp1,
      onStepDown: onStepDown1,
    } = useNumeric({
      initial: value[1],
      stepSize: other.step || 0.1,
      min: other.min == undefined ? -Infinity : other.min,
      max: other.max == undefined ? -Infinity : other.max,
      onValidChange: async (v) =>
        onChange({
          target: {
            name: other.name,
            value: { x: value[0], y: v, z: value[2] },
          },
        }),
    });

    const {
      textValue: textValue2,
      status: status2,
      onChange: onChangeInner2,
      onStepUp: onStepUp2,
      onStepDown: onStepDown2,
    } = useNumeric({
      initial: value[2],
      stepSize: other.step || 0.1,
      min: other.min == undefined ? -Infinity : other.min,
      max: other.max == undefined ? -Infinity : other.max,
      onValidChange: async (v) =>
        onChange({
          target: {
            name: other.name,
            value: { x: value[0], y: value[1], z: v },
          },
        }),
    });

    return (
      <Stack
        direction="row"
        spacing={1}
        ref={ref}
        divider={<Divider orientation="vertical" flexItem />}
      >
        <Input
          size="small"
          disabled={disabled}
          disableUnderline
          className={other.className}
          label={null}
          value={textValue0}
          inputProps={{ step: 0.1 }}
          onFocus={other.onFocus}
          onBlur={other.onBlur}
          onChange={onChangeInner0}
          margin="dense"
          endAdornment={
            <InputAdornment position="end">
              <Spinner
                disabled={disabled}
                above={
                  status0 === NUMERIC_STATUS.ABOVE ||
                  status0 === NUMERIC_STATUS.UPPER_BOUND
                }
                below={
                  status0 === NUMERIC_STATUS.BELOW ||
                  status0 === NUMERIC_STATUS.LOWER_BOUND
                }
                onClickDown={onStepDown0}
                onClickUp={onStepUp0}
              />
            </InputAdornment>
          }
        />
        <Input
          size="small"
          disabled={disabled}
          disableUnderline
          className={other.className}
          label={null}
          value={textValue1}
          inputProps={{ step: 0.1 }}
          onFocus={other.onFocus}
          onBlur={other.onBlur}
          onChange={onChangeInner1}
          margin="dense"
          endAdornment={
            <InputAdornment position="end">
              <Spinner
                disabled={disabled}
                above={
                  status1 === NUMERIC_STATUS.ABOVE ||
                  status1 === NUMERIC_STATUS.UPPER_BOUND
                }
                below={
                  status1 === NUMERIC_STATUS.BELOW ||
                  status1 === NUMERIC_STATUS.LOWER_BOUND
                }
                onClickDown={onStepDown1}
                onClickUp={onStepUp1}
              />
            </InputAdornment>
          }
        />
        <Input
          size="small"
          disabled={disabled}
          disableUnderline
          className={other.className}
          label={null}
          value={textValue2}
          inputProps={{ step: 0.1 }}
          onFocus={other.onFocus}
          onBlur={other.onBlur}
          onChange={onChangeInner2}
          margin="dense"
          endAdornment={
            <InputAdornment position="end">
              <Spinner
                disabled={disabled}
                above={
                  status2 === NUMERIC_STATUS.ABOVE ||
                  status2 === NUMERIC_STATUS.UPPER_BOUND
                }
                below={
                  status2 === NUMERIC_STATUS.BELOW ||
                  status2 === NUMERIC_STATUS.LOWER_BOUND
                }
                onClickDown={onStepDown2}
                onClickUp={onStepUp2}
              />
            </InputAdornment>
          }
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
      <InputLabel
        htmlFor="outlined-position-vector"
        color="primaryColor"
        shrink
      >
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
  value = { x: 0, y: 0, z: 0 },
  onChange = (value) => {},
  disabled = false,
  active = false,
  onToggleActivity = (newValue) => {},
}) {
  return (
    <FormControl>
      <InputLabel
        htmlFor="outlined-position-vector"
        color="primaryColor"
        shrink
      >
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
              onClick={() => onToggleActivity(!active)}
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
