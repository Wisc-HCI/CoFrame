import React from "react";
import { Box } from "grommet";

import useStore from "../../stores/Store";
import { Collapse } from "../Elements/Collapse";
import { NumberInput } from "../Elements/NumberInput";
import shallow from "zustand/shallow";
import { TextField } from "@mui/material";

function JointGripperInput({ robotID, isGripper }) {
  const [initialStateInfo, initialState, initialStateValue] = useStore(
    (state) => {
      if (isGripper) {
        return [[], {}, state.programData[robotID].properties.initialGripState];
      } else {
        let initialStateInfo = [];
        for (const [key, value] of Object.entries(
          state.programData[robotID].properties.initialJointState
        )) {
          const lower =
            state.programData[robotID].properties.jointLimit[key].lower;
          const upper =
            state.programData[robotID].properties.jointLimit[key].upper;

          initialStateInfo.push({
            key,
            value: value,
            lower: lower,
            upper: upper,
          });
        }
        const initialState =
          state.programData[robotID].properties.initialJointState;
        return [initialStateInfo, initialState, null];
      }
    },
    shallow
  );

  const updateItemSimpleProperty = useStore(
    (state) => state.updateItemSimpleProperty,
    shallow
  );

  if (isGripper) {
    return (
      <Collapse defaultOpen header="Initial Gripper State" contentStyle={{paddingTop:'15px'}}>
        <TextField
            label='Distance'
            type="number"
            value={initialStateValue}
            min={0}
            onChange={(value) =>
                updateItemSimpleProperty(robotID, "initialGripState", value)
              }
          />
      </Collapse>
    );
  } else {
    return (
      <Collapse defaultOpen header="Initial Joint States" spacing={2} contentStyle={{paddingTop:'15px'}}>
        {initialStateInfo.map((io, i) => (
          <TextField
            key={io.key}
            label={io.key.replace(/_/g, " ")}
            type="number"
            value={io.value}
            min={io.lower}
            max={io.upper}
            onChange={(e) =>
              updateItemSimpleProperty(robotID, {
                ...initialState,
                [io.key]: e.target.value,
              })
            }
          />
        ))}
      </Collapse>
    );
  }
}
export default JointGripperInput;
