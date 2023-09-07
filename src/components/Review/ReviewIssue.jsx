import React, { memo, useCallback } from "react";
import { FiSlash, FiAlertCircle, FiEye, FiEyeOff } from "react-icons/fi";
import useStore from "../../stores/Store";
import { Card, CardHeader, Avatar, IconButton, Switch, CardContent } from "@mui/material";
import { getPlotInfo } from "../ContextualInfo/Plots";
import { shallow } from 'zustand/shallow';

export const ReviewIssue = memo(({ issueId }) => {
  const issue = useStore(
    useCallback((state) => state.issues[issueId], [issueId])
  );
  const [setFeaturedDocs,setIssueCompletion, primaryColor, addFocusItem, clearFocus, focus, captureFocus] =
    useStore(
      (state) => [
        state.setFeaturedDocs,
        state.setIssueCompletion,
        state.primaryColor,
        state.addFocusItem,
        state.clearFocus,
        state.focus,
        state.captureFocus
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
              disabled={captureFocus}
              onClick={() => {
                if (focused) {
                  clearFocus();
                  setFeaturedDocs({});
                } else {
                  let first = false;
                  issue.focus.forEach((item) => {
                    addFocusItem(item, first);
                    if (!first) {
                      first = true;
                    }
                  });
                  addFocusItem(issueId, true);
                  if (issue.featuredDocs) {
                    const active = issue.focus.length > 0 ? issue.focus[issue.focus.length-1] : null;
                    setFeaturedDocs(issue.featuredDocs,active);
                  }
                }
              }}
            >
              {focused ? <FiEyeOff /> : <FiEye />}
            </IconButton>
            {!issue.requiresChanges && (
              <Switch
                size="small"
                color="primaryColor"
                checked={issue.complete}
                onChange={(e) => {
                  onCheckedChange();
                  e.stopPropagation();
                }}
              />
            )}
          </>
        }
      ></CardHeader>
      {focused && issue.graphData &&  ( //!issue.graphData.isTimeseries &&
        <CardContent style={{padding:0}}>{getPlotInfo({focusItem:issue})}</CardContent>
      )}
    </Card>
  );
});
