import React, { useState, useEffect, memo } from "react";
import { isNumber, isNaN } from "lodash";
import {
  InputAdornment,
  InputLabel,
  FormControl,
  OutlinedInput,
  Typography,
  TextField,
} from "@mui/material";
import styled from "@emotion/styled";
import { FiChevronUp, FiChevronDown } from "react-icons/fi";
// import { useNumberInput } from "./numberHooks";
import {
  NumberInput as ChakraNumberInput,
  NumberInputField,
  NumberInputStepper,
  NumberIncrementStepper,
  NumberDecrementStepper,
  useNumberInput,
  ChakraProvider,
} from "@chakra-ui/react";

export const NumberInput = ({ value, min, max, step, disabled, onChange, precision, label, prefix, suffix, onFocus, onBlur, style }) => {
  // const {} = useNumberInput({ value, isReadOnly: disabled });
  const { getInputProps, getIncrementButtonProps, getDecrementButtonProps } =
    useNumberInput({
      step,
      value,
      min,
      max,
      precision,
      isReadOnly:disabled
    })

    const valid = true;

    const {onChange: innerOnChange, value: innerValue, ...inputProps} = getInputProps();
  console.log({inputProp:getInputProps(),incrementButtonProps:getIncrementButtonProps(),decrementButtonProps:getDecrementButtonProps()})

  return (
    <FormControl
        // onMouseEnter={onMouseEnter}
        // onMouseLeave={onMouseLeave}
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
        <OutlinedInput
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
          value={innerValue}
          onChange={innerOnChange}
          style={{ paddingRight: 4, ...style }}
          inputProps={{ className: "nodrag", ...inputProps }}
          startAdornment={
            <InputAdornment position="start">{prefix}</InputAdornment>
          }
          endAdornment={
            <InputAdornment position="end">
              <Typography variant='caption' style={{marginRight:4}}>{suffix}</Typography>
              <Spinner
                incrementProps={getIncrementButtonProps()}
                deccrementProps={getDecrementButtonProps()}
              />
            </InputAdornment>
          }
        />
      </FormControl>
  )

  return (
    <ChakraProvider>
      <ChakraNumberInput step={step} value={value} min={min} max={max} keepWithinRange onChange={(_,numNumber)=>onChange(numNumber)} precision={2}>
        <NumberInputField />
        <NumberInputStepper>
          <NumberIncrementStepper />
          <NumberDecrementStepper />
        </NumberInputStepper>
      </ChakraNumberInput>
    </ChakraProvider>
  );
};

// export const NumberInput = memo(
//   ({
//     disabled,
//     label,
//     onChange,
//     step,
//     value = 0,
//     onBlur = (_) => {},
//     onFocus = (_) => {},
//     onMouseEnter = (_) => {},
//     onMouseLeave = (_) => {},
//     suffix = "",
//     prefix = "",
//     min = Number.NEGATIVE_INFINITY,
//     max = Number.POSITIVE_INFINITY,
//     style = {}
//   }) => {
//     const [above, setAbove] = useState(false);
//     const [below, setBelow] = useState(false);
//     const valid = !above && !below;
//     const [storedValue, setStoredValue] = useState(0);

//     const setNewFromButton = (change) => {
//       const numericNew = plus(value, change);
//       if (numericNew > max) {
//         setAbove(true);
//         setBelow(false);
//         onChange(max);
//       } else if (numericNew < min) {
//         setAbove(false);
//         setBelow(true);
//         onChange(min);
//       } else {
//         setAbove(false);
//         setBelow(false);
//         onChange(numericNew);
//       }
//     };

//     const setNewFromInput = (event) => {
//       console.log(event);
//       if (event?.nativeEvent?.data) {
//         if (!VALID_CHARS.includes(event.nativeEvent.data)) {
//           return;
//         }
//       }

//       if (event.target.value === "-") {
//         onChange(0);
//         setStoredValue("-");
//         return;
//       }

//       const numericNew = Number(event.target.value);
//       if (!isNumber(numericNew) || isNaN(numericNew)) {
//         return;
//       }

//       if (numericNew > max) {
//         setAbove(true);
//         setBelow(false);
//         onChange(max);
//       } else if (numericNew < min) {
//         setAbove(false);
//         setBelow(true);
//         onChange(min);
//       } else {
//         setAbove(false);
//         setBelow(false);
//         onChange(numericNew);
//         setStoredValue(event.target.value);
//       }
//       return;
//     };

//     useEffect(() => {
//       if (
//         storedValue !== "-" &&
//         storedValue !== "" &&
//         value !== Number(storedValue)
//       ) {
//         setStoredValue(value);
//       }
//     }, [storedValue, value]);

//     return (
//       <FormControl
//         onMouseEnter={onMouseEnter}
//         onMouseLeave={onMouseLeave}
//         className="nodrag"
//       >
//         <InputLabel
//           className="nodrag"
//           htmlFor="outlined-position-vector"
//           color="primary"
//           shrink
//         >
//           {label}
//         </InputLabel>
//         <OutlinedInput
//           notched
//           className="nodrag"
//           size="small"
//           id="outlined-position-vector"
//           label={label}
//           // type='number'
//           color={!valid ? "error" : "primary"}
//           onFocus={onFocus}
//           onBlur={onBlur}
//           disabled={disabled}
//           value={storedValue}
//           onChange={setNewFromInput}
//           style={{ paddingRight: 4, ...style }}
//           inputProps={{ min, max, className: "nodrag" }}
//           startAdornment={
//             <InputAdornment position="start">{prefix}</InputAdornment>
//           }
//           endAdornment={
//             <InputAdornment position="end">
//               <Typography variant='caption' style={{marginRight:4}}>{suffix}</Typography>
//               <Spinner
//                 disabled={disabled}
//                 above={value >= max}
//                 below={value <= min}
//                 onClickDown={() => setNewFromButton(-step)}
//                 onClickUp={() => setNewFromButton(step)}
//               />
//             </InputAdornment>
//           }
//         />
//       </FormControl>
//     );
//   }
// );

const SpinnerButton = styled.button(
  {
    all: "unset",
    display: "flex",
    flexDirection: "column",
    paddingLeft: "0px",
    paddingTop: "2px",
    paddingRight: "0px",
    paddingBottom: "2px",
    margin: "0px",
    alignItems: "center",
    justifyContent: "center",
    borderRadius: "4px",
    lineHeight: 1,
    height: "10px",
    background: "#22222299",
    "&:focus": {
      background: "#222222",
    },
    "&:hover": {
      background: "#222222",
    },
    opacity: `${(props) => (props.disabled ? 0.5 : 1)}`,
  },
  (props) => ({ opacity: props.disabled ? 0.5 : 1 })
);

export const Spinner = ({ incrementProps, decrementProps }) => {
  return (
    <div
      style={{
        marginLeft: 3,
        display: "inline-flex",
        flexDirection: "column",
        borderRadius: 3,
        justifyContent: "center",
        alignItems: "center",
      }}
    >
      <SpinnerButton
        {...incrementProps}
        style={{ borderBottomLeftRadius: 0, borderBottomRightRadius: 0 }}
      >
        <FiChevronUp />
      </SpinnerButton>
      <SpinnerButton
        {...decrementProps}
        style={{ borderTopLeftRadius: 0, borderTopRightRadius: 0 }}
      >
        <FiChevronDown />
      </SpinnerButton>
    </div>
  );
};
