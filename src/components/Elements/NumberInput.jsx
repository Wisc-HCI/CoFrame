import React, { memo } from "react";
import {
  InputAdornment,
  InputLabel,
  FormControl,
  FormHelperText,
  OutlinedInput,
  Typography
} from "@mui/material";
import styled from "@emotion/styled";
import { FiChevronUp, FiChevronDown } from "react-icons/fi";
import { useNumeric, NUMERIC_STATUS } from "./useNumeric";

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
    style = {}
  }) => {
    const {
      textValue,
      status,
      onChange: onChangeInner,
      onStepUp,
      onStepDown
    } = useNumeric({
      initial: value,
      stepSize: step,
      min,
      max,
      onValidChange: onChange
    });

    const errorState = ![
      NUMERIC_STATUS.LOWER_BOUND,
      NUMERIC_STATUS.WITHIN,
      NUMERIC_STATUS.UPPER_BOUND
    ].includes(status);

    return (
      <FormControl
        onMouseEnter={onMouseEnter}
        onMouseLeave={onMouseLeave}
        className="nodrag"
        error={errorState}
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
          color={
            [
              NUMERIC_STATUS.LOWER_BOUND,
              NUMERIC_STATUS.WITHIN,
              NUMERIC_STATUS.UPPER_BOUND
            ].includes(status)
              ? "primary"
              : "error"
          }
          onFocus={onFocus}
          onBlur={onBlur}
          disabled={disabled}
          value={textValue}
          onChange={onChangeInner}
          style={{ paddingRight: 4, ...style }}
          inputProps={{ min, max, className: "nodrag" }}
          startAdornment={
            <InputAdornment position="start">{prefix}</InputAdornment>
          }
          endAdornment={
            <InputAdornment position="end">
              <Typography variant="caption" style={{ marginRight: 4 }}>
                {suffix}
              </Typography>
              <Spinner
                disabled={disabled}
                above={
                  status === NUMERIC_STATUS.ABOVE ||
                  status === NUMERIC_STATUS.UPPER_BOUND
                }
                below={
                  status === NUMERIC_STATUS.BELOW ||
                  status === NUMERIC_STATUS.LOWER_BOUND
                }
                onClickDown={onStepDown}
                onClickUp={onStepUp}
              />
            </InputAdornment>
          }
        />
        {errorState && (
          <FormHelperText id="component-error-text">
            Error: Used Value is {value}
          </FormHelperText>
        )}
      </FormControl>
    );
  }
);

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
      background: "#222222"
    },
    "&:hover": {
      background: "#222222"
    },
    opacity: `${(props) => (props.disabled ? 0.5 : 1)}`
  },
  (props) => ({ opacity: props.disabled ? 0.5 : 1 })
);

export const Spinner = ({ onClickUp, onClickDown, disabled, above, below }) => {
  return (
    <div
      style={{
        marginLeft: 3,
        display: "inline-flex",
        flexDirection: "column",
        borderRadius: 3,
        justifyContent: "center",
        alignItems: "center"
      }}
    >
      <SpinnerButton
        disabled={disabled || above}
        onClick={onClickUp}
        style={{ borderBottomLeftRadius: 0, borderBottomRightRadius: 0 }}
      >
        <FiChevronUp />
      </SpinnerButton>
      <SpinnerButton
        disabled={disabled || below}
        onClick={onClickDown}
        style={{ borderTopLeftRadius: 0, borderTopRightRadius: 0 }}
      >
        <FiChevronDown />
      </SpinnerButton>
    </div>
  );
};
