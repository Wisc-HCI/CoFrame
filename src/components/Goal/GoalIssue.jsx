import React from "react";
import { Badge, Stack, Button, Typography } from "@mui/material";
import { FiCheck, FiX, FiClipboard, FiEye, FiEyeOff } from "react-icons/fi";
import { Blurb } from "../ContextualInfo/Blurb";
import useStore from "../../stores/Store";
import shallow from "zustand/shallow";

export const GoalIssue = ({ goal }) => {
  const [addFocusItem, clearFocus, focus, isProcessing, setActiveTab] = useStore(
    (state) => [
      state.addFocusItem,
      state.clearFocus,
      state.focus,
      state.processes.planProcess !== null &&
        state.processes.planProcess !== undefined,
      state.setActiveTab
    ],
    shallow
  );

  const exampleFocused =
    goal.properties.example.length > 0 &&
    focus.includes(goal.properties.example[0]);

  const Icon = FiClipboard;

  return (
    <Blurb highlight="rgb(50,50,50)">
      <Stack direction="row" spacing={2} style={{ width: "100%" }}>
        <div>
          <Badge
            style={{
              right: 5,
              top: "50%",
              transform: "translateY(-50%)",
            }}
            anchorOrigin={{
              vertical: "bottom",
              horizontal: "right",
            }}
            badgeContent={
              goal.properties.isComplete ? (
                <FiCheck
                  style={{
                    width: 15,
                    height: 15,
                    color: "white",
                  }}
                />
              ) : (
                <FiX
                  style={{
                    width: 15,
                    height: 15,
                    color: "white",
                  }}
                />
              )
            }
            color={goal.properties.isComplete ? "primary" : "error"}
          >
            <Icon style={{ width: 35, height: 35, color: "white" }} />
          </Badge>
        </div>
        <Stack direction="column" spacing={2} style={{ width: "100%" }}>
          <Typography style={{fontSize:16,marginTop:10,marginBottom:0,color:'#ddd'}}>{goal.properties.header}</Typography>
          <Typography style={{fontSize:12,marginTop:4,marginBottom:0,color:'#aaa'}}>{goal.properties.textfield}</Typography>
        </Stack>
        {goal.properties.example.length > 0 && (
          <div>
            <Button
              variant="outlined"
              style={{ top: "50%", transform: "translateY(-50%)" }}
              color={exampleFocused ? "primary" : "vibrant"}
              onClick={() => {
                if (exampleFocused) {
                  clearFocus();
                  setActiveTab({id: "default"})
                } else {
                  // Update the simulation
                  addFocusItem(goal.properties.example[0], false);
                  // Update the program tab
                  setActiveTab({id: goal.properties.tab});
                }
              }}
              startIcon={exampleFocused ? <FiEyeOff /> : <FiEye />}
              disabled={isProcessing}
            >
              Preview
            </Button>
          </div>
        )}
      </Stack>
    </Blurb>
  );
};
