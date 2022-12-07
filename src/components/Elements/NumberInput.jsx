import React, {useState, useEffect, memo} from 'react';
import { isNumber, isNaN } from "lodash";
import {
    InputAdornment,
    InputLabel,
    FormControl,
  } from "@mui/material";

// export const NumberInput = ({ value, min, max, onChange, disabled, style, visualScaling }) => {
//     const usedScaling = visualScaling ? visualScaling : 1;
//     return (
//         <TextInput
//             disabled={disabled}
//             value={value*usedScaling}
//             type='number'
//             step={0.1}
//             textAlign="center"
//             style={{fontSize:14,color: toNumber(value) >= min && toNumber(value) <= max ? style?.color : 'red'}}
//             onChange={event => {onChange(toNumber(event.target.value)/usedScaling)}}
//         />
//     )
// }

export const NumberInput = memo(
    ({
      disabled,
      label,
      onChange,
      step,
      value = 0,
      onBlur = (_) => {},
      onFocus = (_) => {},
      onMouseEnter = (_) => {},
      onMouseLeave = (_) => {},
      suffix = "",
      prefix = "",
      min = Number.NEGATIVE_INFINITY,
      max = Number.POSITIVE_INFINITY,
    }) => {
      const [above, setAbove] = useState(false);
      const [below, setBelow] = useState(false);
      const valid = !above && !below;
      const [storedValue, setStoredValue] = useState(0);
  
      const setNewFromButton = (change) => {
        const numericNew = plus(value, change);
        if (numericNew > max) {
          setAbove(true);
          setBelow(false);
          onChange(max);
        } else if (numericNew < min) {
          setAbove(false);
          setBelow(true);
          onChange(min);
        } else {
          setAbove(false);
          setBelow(false);
          onChange(numericNew);
        }
      };
  
      const setNewFromInput = (event) => {
        console.log(event);
        if (event?.nativeEvent?.data) {
          if (!VALID_CHARS.includes(event.nativeEvent.data)) {
            return;
          }
        }
  
        if (event.target.value === "-") {
          onChange(0);
          setStoredValue("-");
          return;
        }
  
        const numericNew = Number(event.target.value);
        if (!isNumber(numericNew) || isNaN(numericNew)) {
          return;
        }
  
        if (numericNew > max) {
          setAbove(true);
          setBelow(false);
          onChange(max);
        } else if (numericNew < min) {
          setAbove(false);
          setBelow(true);
          onChange(min);
        } else {
          setAbove(false);
          setBelow(false);
          onChange(numericNew);
          setStoredValue(event.target.value);
        }
        return;
      };
  
      useEffect(() => {
        if (
          storedValue !== "-" &&
          storedValue !== "" &&
          value !== Number(storedValue)
        ) {
          setStoredValue(value);
        }
      }, [storedValue, value]);
  
      return (
        <FormControl
          onMouseEnter={onMouseEnter}
          onMouseLeave={onMouseLeave}
          className="nodrag"
        >
          <InputLabel
            className="nodrag"
            htmlFor="outlined-position-vector"
            color="primary"
            shrink
          >
            {label}
          </InputLabel>
          <OutlinedNumberInput
            notched
            className="nodrag"
            size="small"
            id="outlined-position-vector"
            label={label}
            // type='number'
            color={!valid ? "error" : "primary"}
            onFocus={onFocus}
            onBlur={onBlur}
            disabled={disabled}
            value={storedValue}
            onChange={setNewFromInput}
            style={{ paddingRight: 4 }}
            inputProps={{ min, max, className: "nodrag" }}
            startAdornment={
              <InputAdornment position="start">{prefix}</InputAdornment>
            }
            endAdornment={
              <InputAdornment position="end">
                {suffix}
                <Spinner
                  disabled={disabled}
                  above={value >= max}
                  below={value <= min}
                  onClickDown={() => setNewFromButton(-step)}
                  onClickUp={() => setNewFromButton(step)}
                />
              </InputAdornment>
            }
          />
        </FormControl>
      );
    }
  );