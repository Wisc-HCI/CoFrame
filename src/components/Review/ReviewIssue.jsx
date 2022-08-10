import React, { useCallback } from "react";
// import { Space, List, Avatar, Switch, Button } from 'antd';
import { Box, Text } from "grommet";
// import { EllipsisOutlined, StopOutlined, ExclamationOutlined } from '@ant-design/icons';
import { FiSlash, FiAlertCircle, FiEye, FiEyeOff } from "react-icons/fi";
import useStore from "../../stores/Store";
// import { Switch } from "../Elements/Switch";
// import { FancyText } from "../Elements/FancyText";
import Switch from "@mui/material/Switch";
import FormControlLabel from "@mui/material/FormControlLabel";
import { Card, CardHeader, Avatar, IconButton } from "@mui/material";
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

  return (
    <Box
      onClick={() => {
        let first = false;
        issue.focus.forEach((item) => {
          addFocusItem(item, first);
          if (!first) {
            first = true;
          }
        });
        addFocusItem(issueId, true);
      }}
      focusIndicator={false}
      direction="row"
      flex
      round="xsmall"
      align="center"
      justify="between"
      gap="small"
      pad={{ top: "10px", bottom: "10px", left: "7px", right: "7px" }}
      background={focus.includes(issueId) ? `${primaryColor}22` : "#2f2f2f"}
      style={{
        boxShadow: focus.includes(issueId)
          ? `inset 0pt 0pt 1pt 1pt ${primaryColor}`
          : null,
      }}
    >
      <Box direction="row" align="center" gap="small" flex>
        <Avatar
          size="small"
          background={issue.requiresChanges ? "#a61d24" : "grey"}
        >
          <Icon style={{ color: "white" }} />
        </Avatar>
        <Box direction="column" style={{ maxWidth: 200 }} pad="xsmall">
          <Text>{issue.title}</Text>
          <Text size="small">
            {issue.requiresChanges
              ? "Please reconfigure the item to remove this error"
              : "Check to dismiss"}
          </Text>
        </Box>
      </Box>

      <Box
        direction="row"
        align="center"
        onClick={(e) => e.stopPropagation()}
        focusIndicator={false}
      >
        {!issue.requiresChanges && (
          <FormControlLabel
            value={issue.complete}
            control={
              <Switch
                size="small"
                color="primaryColor"
                disabled={issue.requiresChanges}
                value={issue.complete}
                onChange={onCheckedChange}
              />
            }
            label={issue.complete ? "Done!" : ""}
            labelPlacement="start"
          />
          // <Switch
          //   label={
          //     issue.complete ? (
          //       <FancyText css={{ color: primaryColor, fontSize:10 }}>Done!</FancyText>
          //     ) : null
          //   }
          //   disabled={issue.requiresChanges}
          //   value={issue.complete}
          //   onCheckedChange={onCheckedChange}
          //   highlightColor={primaryColor}
          // />
        )}
        {/* {issue.focus && <Button onClick={()=>{addFocusItem(issue.focus.id, true);addFocusItem(issueId, true)}} icon={<FiMoreHorizontal/>}/>} */}
      </Box>
    </Box>
  );
};
