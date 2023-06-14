import { useState, useEffect, startTransition } from "react";
import { isNumber, isNaN } from "lodash";
import { strip } from "number-precision";

export const NUMERIC_STATUS = {
  BELOW: "BELOW",
  LOWER_BOUND: "LOWER",
  WITHIN: "WITHIN",
  UPPER_BOUND: "UPPER",
  ABOVE: "ABOVE",
  INVALID: "INVALID",
};

const PASSABLE_NUMERIC_STATUSES = [
  NUMERIC_STATUS.LOWER_BOUND,
  NUMERIC_STATUS.WITHIN,
  NUMERIC_STATUS.UPPER_BOUND,
];

const VALID_CHARS = [
  "0",
  "1",
  "2",
  "3",
  "4",
  "5",
  "6",
  "7",
  "8",
  "9",
  ".",
  "-",
  null,
];

export const getStatus = (v, min, max) => {
  if (v < max && v > min) {
    return NUMERIC_STATUS.WITHIN;
  } else if (v > max) {
    return NUMERIC_STATUS.ABOVE;
  } else if (v === max) {
    return NUMERIC_STATUS.UPPER_BOUND;
  } else if (v === min) {
    return NUMERIC_STATUS.LOWER_BOUND;
  } else if (v < min) {
    return NUMERIC_STATUS.BELOW;
  }
};

export const parse = (v, min, max) => {
  if (typeof v === "number") {
    return {
      value: v,
      status: getStatus(v, min, max),
      textValue: v.toString(),
    };
  } else if (typeof v === "string") {
    const parsed = Number(v);
    if (v === "-") {
      return { value: 0, status: getStatus(0, min, max), textValue: v };
    }
    if (isNumber(parsed) && !isNaN(parsed)) {
      return {
        value: parsed,
        status: getStatus(parsed, min, max),
        textValue: v,
      };
    }
  }
  console.log("failed with ", v, typeof v);
};

// State: { value, status, textValue, min, max, step }
export const NumberReducer = (state, action) => {
  if (action.type === "increment") {
    console.log("STATE", state);
    const incrementedState = parse(
      strip(state.value + state.step),
      state.min,
      state.max
    );
    console.log(
      `incrementing ${state.value} + ${state.step}=>${incrementedState.value}`
    );
    return { ...state, ...incrementedState };
  } else if (action.type === "decrement") {
    const decrementedState = parse(
      strip(state.value - state.step),
      state.min,
      state.max
    );
    console.log(
      `decrementing ${state.value} - ${state.step}=>${decrementedState.value}`
    );
    return { ...state, ...decrementedState };
  } else if (action.type === "change") {
    if (typeof action.value === "number") {
      const { value, status, textValue } = parse(
        action.value,
        state.min,
        state.max
      );
      return { ...state, value, status, textValue };
    } else if (!VALID_CHARS.includes(action.change)) {
      console.log("action change", action.change);
      return state;
    } else if (action.value === "-" || action.value === "") {
      const { value, status, textValue } = parse(
        action.value,
        state.min,
        state.max
      );
      return { ...state, value, status, textValue };
    } else {
      const result = parse(action.value, state.min, state.max);
      if (!result) {
        return {
          ...state,
          textValue: action.value,
          status: NUMERIC_STATUS.INVALID,
        };
      } else if (
        PASSABLE_NUMERIC_STATUSES.includes(result.status) ||
        result.status === NUMERIC_STATUS.BELOW ||
        result.status === NUMERIC_STATUS.ABOVE
      ) {
        return {
          ...state,
          value: result.value,
          status: result.status,
          textValue: action.value,
        };
      }
    }
  } else {
    return state;
  }
};

// State: {vector: [{ value, status, textValue }, min, max, step]}
export const VectorReducer = (state, action) => {
  if (action.type === "increment") {
    // console.log("STATE", state);
    const incrementedState = parse(
      strip(state.vector[action.idx].value + state.step),
      state.min,
      state.max
    );
    // console.log(
    //   `incrementing ${state.vector[action.idx].value} + ${state.step} @ ${action.idx} => ${incrementedState.value}`
    // );
    let vector = [...state.vector];
    vector[action.idx] = incrementedState;
    return { ...state, vector };
  } else if (action.type === "decrement") {
    const decrementedState = parse(
      strip(state.vector[action.idx].value - state.step),
      state.min,
      state.max
    );
    // console.log(
    //   `decrementing ${state.vector[action.idx].value} - ${state.step} @ ${action.idx} => ${decrementedState.value}`
    // );
    let vector = [...state.vector];
    vector[action.idx] = decrementedState;
    return { ...state, vector };
  } else if (action.type === "change") {
    if (typeof action.value === "number") {
      const vectorValue = parse(
        action.value,
        state.min,
        state.max
      );
      let vector = [...state.vector];
      vector[action.idx] = vectorValue;
      return { ...state, vector };
    } else if (!VALID_CHARS.includes(action.change)) {
      // console.log("action change", action.change);
      return state;
    } else if (action.value === "-" || action.value === "") {
      const vectorValue = parse(
        action.value,
        state.min,
        state.max
      );
      let vector = [...state.vector];
      vector[action.idx] = vectorValue;
      return { ...state, vector };
    } else {
      const result = parse(action.value, state.min, state.max);
      if (!result) {
        const vectorValue = {...state.vector[action.idx], textValue: action.value, status: NUMERIC_STATUS.INVALID};
        let vector = [...state.vector];
        vector[action.idx] = vectorValue;
        return {
          ...state,
          vector,
        };
      } else if (
        PASSABLE_NUMERIC_STATUSES.includes(result.status) ||
        result.status === NUMERIC_STATUS.BELOW ||
        result.status === NUMERIC_STATUS.ABOVE
      ) {
        let vector = [...state.vector];
        vector[action.idx] = result;
        return {
          ...state,
          vector,
        };
      }
    }
  } else {
    return state;
  }
};

export const useNumeric = ({
  initial = 0,
  stepSize = 1,
  min = Number.NEGATIVE_INFINITY,
  max = Number.POSITIVE_INFINITY,
  onValidChange = () => {},
}) => {
  const [state, setState] = useState(parse(initial));

  const onChange = (event) => {
    startTransition(() => {
      console.log("onchange", event.target.value);
      if (event?.nativeEvent?.data) {
        if (!VALID_CHARS.includes(event.nativeEvent.data)) {
          setState((prev) => ({
            value: prev.value,
            status: prev.status,
            textValue: prev.textValue,
          }));
          console.log("finished setstate");
          return;
        }
      }
      if (event.target.value === "-") {
        console.log("dash input");
        setState({
          value: 0,
          status: getStatus(0, min, max),
          textValue: event.target.value,
        });
        console.log("finished setstate");
        return;
      }

      if (event.target.value === "") {
        console.log("empty input");
        setState({
          value: 0,
          status: getStatus(0, min, max),
          textValue: event.target.value,
        });
        console.log("finished setstate");
        return;
      }

      const newState = parse(event.target.value, min, max);
      if (!newState) {
        setState((prev) => ({
          value: prev.value,
          status: NUMERIC_STATUS.INVALID,
          textValue: event.target.value,
        }));
        console.log("finished setstate");
        return;
      } else if (
        PASSABLE_NUMERIC_STATUSES.includes(newState.status) ||
        newState.status === NUMERIC_STATUS.BELOW ||
        newState.status === NUMERIC_STATUS.ABOVE
      ) {
        setState(newState);
        console.log("finished setstate");
        return;
      }
      console.log("not handled", event.target.value);
    });
  };

  const onStepUp = () => {
    const newState = parse(strip(state.value + stepSize), min, max);
    setState(newState);
  };

  const onStepDown = () => {
    const newState = parse(strip(state.value - stepSize), min, max);
    setState(newState);
  };

  useEffect(() => {
    console.log("useEffect", state.value);
    if (PASSABLE_NUMERIC_STATUSES.includes(state.status)) {
      onValidChange(state.value);
    } else if (state.status === NUMERIC_STATUS.BELOW) {
      onValidChange(min);
    } else if (state.status === NUMERIC_STATUS.ABOVE) {
      onValidChange(max);
    }
  }, [state.value, state.status, min, max, onValidChange]);

  return {
    textValue: state.textValue,
    status: state.status,
    onChange,
    onStepUp,
    onStepDown,
  };
};
