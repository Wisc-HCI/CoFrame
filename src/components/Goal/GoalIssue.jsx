import React from "react";
import { Card, Avatar, useTheme, Typography } from "@mui/material";
import { FiCheck, FiX, FiClipboard } from "react-icons/fi";
import { Box } from "grommet";

export const GoalIssue = ({ goalText, isComplete }) => {
  const theme = useTheme();

  const Icon = FiClipboard;
  
  return (
    <Card
      raised
      sx={{
        backgroundColor: "#333",
        borderRadius: 2,
        boxShadow: null,
      }}
    >
      <Box direction="row" style={{padding: 10, display: "flex", flexDirection: "row"}}>
        <Box direction="row" style={{display: "flex", flexDirection: "row", paddingRight: 30}}>
            <Avatar
              sx={{ 
                backgroundColor: "grey",
                position: "relative",
                top: "50%",
                transform: "translateY(-50%)"
              }}
            >
            <Icon style={{ color: "white" }} />
          </Avatar>
          <Typography component="div" style={{paddingLeft: 15}}>
          {goalText}
          </Typography>
        </Box>
        <Box direction="row" style={{marginLeft: "auto", marginRight: 15}}>
          <Avatar
            sx={{
              width: 28,
              height: 28,
              backgroundColor: theme.palette.quiet.main,
              boxShadow: `0px 0px 2px 2px ${theme.palette.primary.main}`,
              position: "relative",
              top: "50%",
              transform: "translateY(-50%)"
            }}
            color="primary"
          >
          {isComplete ? 
          <FiCheck style={{ width: 20, height: 20, color: theme.palette.primary.main }} /> :
          <FiX style={{ width: 20, height: 20, color: theme.palette.primary.main }} /> }
          </Avatar>
        </Box>
      </Box>
    </Card>
  );
};
