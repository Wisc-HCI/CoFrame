import React, { memo } from "react";
import { Box } from "grommet";
import { GoalIssue } from "./GoalIssue";
import { Collapse } from "../Elements/Collapse";
import { Typography } from "@mui/material";

export const GoalSection = memo(({goalList}) => {

  return (
      <div>
        {goalList.length > 0 ? (
          <>
            {goalList.map((goal) => (
              <GoalIssue key={goal.id} goal={goal}/>
            ))}
          </>
        ) : (
          <Typography sx={{ textAlign: "center" }}>No Task Goals</Typography>
        )}
      </div>
  );
});
