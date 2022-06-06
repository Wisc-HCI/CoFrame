import React, { useCallback } from "react";
// import { Space, List, Avatar, Switch, Button } from 'antd';
import { Box, Avatar, Text } from "grommet";
// import { EllipsisOutlined, StopOutlined, ExclamationOutlined } from '@ant-design/icons';
import { FiSlash, FiAlertCircle } from "react-icons/fi";
import useStore from "../../stores/Store";
import { Switch } from "../Elements/Switch";
import { FancyText } from "../Elements/FancyText";

export const ReviewIssue = ({ issueId }) => {
  const issue = useStore(
    useCallback((state) => state.issues[issueId], [issueId])
  );
  const [setIssueCompletion, primaryColor, addFocusItem, focus] = useStore(
    (state) => [
      state.setIssueCompletion,
      state.primaryColor,
      state.addFocusItem,
      state.focus,
    ]
  );

  const Icon = issue.requiresChanges ? FiSlash : FiAlertCircle;

  const onCheckedChange = () => {
    if (!issue.requiresChanges) {
      setIssueCompletion(issueId, !issue.complete);
    }
  };

  return (
    <Box
      onClick={() => {
        let first = false;
        issue.focus.forEach(item => {
          addFocusItem(item, first);
          if (!first) {
            first = true;
          }
        })
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
      <Box direction='row' align='center' gap='small' flex>
        <Avatar
          size="small"
          background={issue.requiresChanges ? "#a61d24" : "grey"}
        >
          <Icon style={{ color: "white" }} />
        </Avatar>
        <Box direction="column" style={{maxWidth:200}}>
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
          <Switch
            label={
              issue.complete ? (
                <FancyText css={{ color: primaryColor, fontSize:10 }}>Done!</FancyText>
              ) : null
            }
            disabled={issue.requiresChanges}
            value={issue.complete}
            onCheckedChange={onCheckedChange}
            highlightColor={primaryColor}
          />
        )}
        {/* {issue.focus && <Button onClick={()=>{addFocusItem(issue.focus.id, true);addFocusItem(issueId, true)}} icon={<FiMoreHorizontal/>}/>} */}
      </Box>
    </Box>
  );
};
