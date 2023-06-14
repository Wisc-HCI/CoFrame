import React, { forwardRef, memo, useEffect, useReducer } from "react";
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
import { Spinner } from "./NumberInput";
import {
  NUMERIC_STATUS,
  NumberReducer,
  parse,
  getStatus,
  VectorReducer,
} from "./useNumeric";
import PropTypes from "prop-types";
import { useTheme } from "@emotion/react";

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
        ...other
      },
      ref
    ) => {
      // console.log("rerendering compound input", value);

      const theme = useTheme();

      const [state, dispatch] = useReducer(
        VectorReducer,
        VectorReducer(
          {
            vector: [
              {
                value: value[0],
                status: getStatus(value[0], min, max),
                textValue: String(value[0]),
              },
              {
                value: value[1],
                status: getStatus(value[1], min, max),
                textValue: String(value[1]),
              },
              {
                value: value[0],
                status: getStatus(value[2], min, max),
                textValue: String(value[2]),
              },
            ],
            min,
            max,
            step,
          },
          {}
        )
      );

      useEffect(() => {
        if (
          [
            NUMERIC_STATUS.WITHIN,
            NUMERIC_STATUS.LOWER_BOUND,
            NUMERIC_STATUS.UPPER_BOUND,
          ].includes(state.vector[0].status) &&
          [
            NUMERIC_STATUS.WITHIN,
            NUMERIC_STATUS.LOWER_BOUND,
            NUMERIC_STATUS.UPPER_BOUND,
          ].includes(state.vector[1].status) &&
          [
            NUMERIC_STATUS.WITHIN,
            NUMERIC_STATUS.LOWER_BOUND,
            NUMERIC_STATUS.UPPER_BOUND,
          ].includes(state.vector[2].status)
        ) {
          // console.log(
          //   "dispatching change",
          //   state.vector[0].value,
          //   state.vector[1].value,
          //   state.vector[2].value
          // );
          onChange(state.vector.map((v) => v.value));
        }
      }, [
        state.vector,
      ]);

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
            value={state.vector[0].textValue}
            onFocus={other.onFocus}
            onBlur={other.onBlur}
            onChange={(e) =>
              dispatch({ type: "change", value: e.target.value, idx: 0 })
            }
            inputProps={{
              style: {
                paddingTop: 7,
                color: [
                  NUMERIC_STATUS.LOWER_BOUND,
                  NUMERIC_STATUS.WITHIN,
                  NUMERIC_STATUS.UPPER_BOUND,
                ].includes(state.vector[0].status)
                  ? null
                  : theme.palette.error,
              },
            }}
            margin="dense"
            style={{ display: "inline-flex" }}
            endAdornment={
              <InputAdornment position="end">
                <Spinner
                  disabled={disabled}
                  above={
                    state.vector[0].status === NUMERIC_STATUS.ABOVE ||
                    state.vector[0].status === NUMERIC_STATUS.UPPER_BOUND
                  }
                  below={
                    state.vector[0].status === NUMERIC_STATUS.BELOW ||
                    state.vector[0].status === NUMERIC_STATUS.LOWER_BOUND
                  }
                  onClickDown={() => dispatch({ type: "decrement", idx: 0 })}
                  onClickUp={() => dispatch({ type: "increment", idx: 0})}
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
            value={state.vector[1].textValue}
            onFocus={other.onFocus}
            onBlur={other.onBlur}
            onChange={(e) =>
              dispatch({ type: "change", value: e.target.value, idx: 1 })
            }
            inputProps={{ style: { paddingTop: 7 } }}
            margin="dense"
            style={{ display: "inline-flex" }}
            endAdornment={
              <InputAdornment position="end">
                <Spinner
                  disabled={disabled}
                  above={
                    state.vector[1].status === NUMERIC_STATUS.ABOVE ||
                    state.vector[1].status === NUMERIC_STATUS.UPPER_BOUND
                  }
                  below={
                    state.vector[1].status === NUMERIC_STATUS.BELOW ||
                    state.vector[1].status === NUMERIC_STATUS.LOWER_BOUND
                  }
                  onClickDown={() => dispatch({ type: "decrement", idx: 1 })}
                  onClickUp={() => dispatch({ type: "increment", idx: 1 })}
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
            value={state.vector[2].textValue}
            onFocus={other.onFocus}
            onBlur={other.onBlur}
            onChange={(e) =>
              dispatch({ type: "change", value: e.target.value, idx: 2 })
            }
            inputProps={{ style: { paddingTop: 7 } }}
            margin="dense"
            style={{ display: "inline-flex" }}
            endAdornment={
              <InputAdornment position="end">
                <Spinner
                  disabled={disabled}
                  above={
                    state.vector[2].status === NUMERIC_STATUS.ABOVE ||
                    state.vector[2].status === NUMERIC_STATUS.UPPER_BOUND
                  }
                  below={
                    state.vector[2].status === NUMERIC_STATUS.BELOW ||
                    state.vector[2].status === NUMERIC_STATUS.LOWER_BOUND
                  }
                  onClickDown={() => dispatch({ type: "decrement", idx: 2 })}
                  onClickUp={() => dispatch({ type: "increment", idx: 2 })}
                />
              </InputAdornment>
            }
          />
        </Stack>
      );
    }
  )
);

// function PositionInput({ itemID, disabled, mode }) {
//   return <Card variant='outline'></Card>;
// }

export const VectorInput = memo(
  ({
    label = "Vector",
    value = [0, 0, 0],
    onChange = () => {},
    disabled = false,
    active = false,
    onToggleActivity = (newValue) => {},
  }) => {
    // console.log("VECTOR VALUE", value);
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
