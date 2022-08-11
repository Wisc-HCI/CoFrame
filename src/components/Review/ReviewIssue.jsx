import React, { useCallback } from "react";
import { FiSlash, FiAlertCircle, FiEye, FiEyeOff } from "react-icons/fi";
import useStore from "../../stores/Store";
import { Card, CardHeader, Avatar, IconButton, Switch } from "@mui/material";
import shallow from "zustand/shallow";

export const ReviewIssue = ({ issueId }) => {
  const issue = useStore(
    useCallback((state) => state.issues[issueId], [issueId])
  );
  const [setIssueCompletion, primaryColor, addFocusItem, clearFocus, focus] =
    useStore(
      (state) => [
        state.setIssueCompletion,
        state.primaryColor,
        state.addFocusItem,
        state.clearFocus,
        state.focus,
      ],
      shallow
    );

  const Icon = issue.requiresChanges ? FiSlash : FiAlertCircle;

  const onCheckedChange = () => {
    if (!issue.requiresChanges) {
      setIssueCompletion(issueId, !issue.complete);
    }
  };

  const focused = focus.includes(issueId);

  return (
    <Card
      raised
      // variant="outlined"
      sx={{
        backgroundColor: "#333",
        borderRadius: 2,
        boxShadow: focused ? `0px 0px 2px 2px ${primaryColor}` : null,
      }}
    >
      <CardHeader
        title={issue.title}
        subheader={
          issue.requiresChanges
            ? "Please reconfigure the item to remove this error"
            : "Check to dismiss"
        }
        avatar={
          <Avatar
            sx={{ backgroundColor: issue.requiresChanges ? "#a61d24" : "grey" }}
          >
            <Icon style={{ color: "white" }} />
          </Avatar>
        }
        action={
          <>
            <IconButton
              color={focused ? 'primary' : 'vibrant'}
              size="small"
              onClick={() => {
                if (focused) {
                  clearFocus();
                } else {
                  let first = false;
                  issue.focus.forEach((item) => {
                    addFocusItem(item, first);
                    if (!first) {
                      first = true;
                    }
                  });
                  addFocusItem(issueId, true);
                }
              }}
            >
              {focused ? <FiEyeOff /> : <FiEye />}
            </IconButton>
            {!issue.requiresChanges && (
              <Switch
                size="small"
                color="primaryColor"
                value={issue.complete}
                onChange={(e) => {
                  onCheckedChange();
                  e.stopPropagation();
                }}
              />
            )}
          </>
        }
      ></CardHeader>
    </Card>
  );
};
