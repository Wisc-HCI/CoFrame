import React from "react";
import { NumberInput } from "./NumberInput";
import { Card, Slider, Typography, Stack } from "@mui/material";
import { clamp } from "lodash";
import { round } from "number-precision";

export const JointInput = ({
  jointState,
  robotJointInfo,
  onChange,
  disabled,
}) => {
  
  return (
    <Card style={{ backgroundColor: "#000", padding: 10 }}>
      <Stack spacing={1}>
        {Object.keys(robotJointInfo).map((key) => (
          <Stack key={key} direction="row" justifyContent="space-between">
            <Typography variant="body2">
              {key.replace(/_/g, " ").replace("joint", "")}
            </Typography>
            <Stack direction="row" spacing={2}>
              <Slider
                aria-label={`${key} Joint Value`}
                value={jointState[key] || 0}
                min={robotJointInfo[key].lower}
                max={robotJointInfo[key].upper}
                style={{ width: 150 }}
                step={0.01}
                disabled={disabled}
                onChange={(e) =>{
                  console.log(Number(e.target.value));
                  onChange({
                    ...jointState,
                    [key]: clamp(
                      Number(e.target.value),
                      robotJointInfo[key].lower,
                      robotJointInfo[key].upper
                    ),
                  })}
                }
              />
              <Typography variant="caption" style={{width:35}}>
                {jointState[key] ? round(jointState[key], 2) : 0}
              </Typography>
            </Stack>
          </Stack>
        ))}
      </Stack>
    </Card>
  );
};
