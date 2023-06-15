import React, {
  forwardRef,
  memo
} from "react";
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
  debounce,
} from "@mui/material";
import PropTypes from "prop-types";
import { round } from 'number-precision';

const CompoundInput = memo(
  forwardRef(
    (
      {
        onChange = () => {},
        value = [0, 0, 0],
        disabled = false,
        step = 0.1,
        min = Number.NEGATIVE_INFINITY,
        max = Number.POSITIVE_INFINITY,
        active = true,
        ...other
      },
      ref
    ) => {
      console.log("rerendering compound input", value);

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
            value={value[0]}
            onChange={debounce((e) =>{
              onChange([round(e.target.value,3),value[1], value[2]])}
            )}
            type='number'
            inputProps={{
              min,max,step
            }}
            margin="dense"
          />
          <Input
            size="small"
            disabled={disabled}
            disableUnderline
            className={other.className}
            label={null}
            value={Number(value[1]).toFixed(3)}
            onChange={debounce((e) =>
              onChange([value[0], round(e.target.value,3),value[2]])
            )}
            type='number'
            inputProps={{
              min,max,step
            }}
            margin="dense"
            style={{ display: "inline-flex" }}
          />
          <Input
            size="small"
            disabled={disabled}
            disableUnderline
            className={other.className}
            label={null}
            value={Number(value[2]).toFixed(3)}
            onChange={debounce((e) =>
              onChange([value[0], value[1], round(e.target.value,3)])
            )}
            type='number'
            inputProps={{
              min,max,step
            }}
            margin="dense"
            style={{ display: "inline-flex" }}
          />
        </Stack>
      );
    }
  )
);


export const VectorInput = memo(
  ({
    label = "Vector",
    value = [0, 0, 0],
    onChange = () => {},
    disabled = false,
    active = false,
    onToggleActivity = (newValue) => {},
  }) => {
    return (
      <FormControl>
        <InputLabel htmlFor="outlined-position-vector" color="primary" shrink>
          {label}
        </InputLabel>
        <OutlinedInput
          notched
          size="small"
          id="outlined-position-vector"
          label={label}
          color="primary"
          disabled={disabled || !active}
          value={value}
          inputComponent={CompoundInput}
          onChange={onChange}
          inputProps={{ active }}
          endAdornment={
            <InputAdornment position="end">
              <IconButton
                size="small"
                disabled={disabled}
                color="primary"
                aria-label="toggle editing position"
                onClick={() => onToggleActivity(!active)}
              >
                {active ? <FiSave /> : <FiEdit2 />}
              </IconButton>
            </InputAdornment>
          }
        />
      </FormControl>
    );
  }
);

VectorInput.propTypes = {
  value: PropTypes.arrayOf(PropTypes.number),
  onChange: PropTypes.func,
  onToggleActivity: PropTypes.func,
  disabled: PropTypes.bool,
  active: PropTypes.bool,
  label: PropTypes.string,
};

export default VectorInput;
