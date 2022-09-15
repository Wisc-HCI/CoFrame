import React, { memo } from "react";
import { Box } from "grommet";
import { GoalIssue } from "./GoalIssue";
import { Collapse } from "../Elements/Collapse";
import { Typography } from "@mui/material";

export const GoalSection = memo(({goalList}) => {

  return (
    <Box direction="column" width="100%">
      <Collapse
        defaultOpen={true}
        header={"Task Goals"}
      >
        {goalList.length > 0 ? (
          <>
            {goalList.map((goal) => (
              <GoalIssue key={goal.id} goalText={goal.properties.textfield} isComplete={goal.properties.isComplete}/>
            ))}
          </>
        ) : (
          <Typography sx={{ textAlign: "center" }}>No Task Goals</Typography>
        )}
      </Collapse>
    </Box>
  );
});
