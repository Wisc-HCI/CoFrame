import { FormControl, Select, InputLabel, MenuItem } from "@mui/material";
import useStore from "../../stores/Store";
import { pickBy } from "lodash";
import { shallow } from "zustand/shallow";
import { DATA_TYPES } from "simple-vp";
import { useState } from "react";

export const PoseCopier = ({ disabled, onSelect = () => {} }) => {
  const poses = useStore(
    (state) =>
      pickBy(
        state.programData,
        (value) =>
          ["waypointType", "locationType"].includes(value.type) &&
          value.dataType === DATA_TYPES.INSTANCE
      ),
    shallow
  );
  const [selection, setSelection] = useState("");

  return (
    <FormControl fullWidth>
      <InputLabel id="pose-select-label">Copy From Pose</InputLabel>
      <Select
        disabled={disabled}
        labelId="pose-select-label"
        id="pose-select"
        label="Copy From Pose"
        value={selection}
        onChange={(e) => {
          console.log(e.target.value);
          setSelection(e.target.value);
          if (e.target.value) {
            onSelect(poses[e.target.value]);
          }
        }}
      >
        {Object.keys(poses).map((key) => (
          <MenuItem key={key} value={key}>
            {poses[key].name}
          </MenuItem>
        ))}
      </Select>
    </FormControl>
  );
};
