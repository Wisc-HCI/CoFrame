import React, { memo, useCallback } from "react";
import { Box } from "grommet";
import { ReviewIssue } from "./ReviewIssue";
import useStore from "../../stores/Store";
import { FrameButton } from "../FrameButton";
import frameStyles from "../../frameStyles";
import { Collapse } from "../Elements/Collapse";
import { Avatar, Chip, Typography, useTheme } from "@mui/material";
import shallow from "zustand/shallow";
import { FiCheck} from "react-icons/fi";

// import './custom.css'

const isComplete = (state, sectionId) =>
  state.sections[sectionId].issues
    .map((issueId) => state.issues[issueId])
    .filter((issue) => !issue.complete).length === 0;

const sectionFrame = (sectionId, frames) =>
  Object.values(frames).filter(
    (filter) => filter.sections.indexOf(sectionId) > -1
  )[0].key;

export const ReviewSection = memo(({ sectionId, blocked, initialBlocked }) => {
  const theme = useTheme();
  const [frame, setFrame] = useStore(
    (state) => [state.frame, state.setFrame],
    shallow
  );

  const [frames, frameNames] = useStore(
    (state) => [state.frames, state.frameNames],
    shallow
  );

  const [
    name,
    issueIds,
    incompleteIds,
    dependencies,
    depNames,
    depFrames,
  ] = useStore(
    useCallback(
      (state) => {
        const name = state.sections[sectionId].name;
        const issueIds = state.sections[sectionId].issues;
        const incompleteIds = issueIds
          .map((issueId) => state.issues[issueId])
          .filter((issue) => !issue.complete);

        const dependencies = initialBlocked
          ? state.sections[sectionId].dependencies.filter(
              (dep) => !isComplete(state, dep)
            )
          : [];
        // console.log(issueIds.map((id) => state.issues[id]));
        const depNames = dependencies.map((dep) => state.sections[dep].name);
        const depFrames = dependencies.map((dep) =>
          sectionFrame(dep, state.frames)
        );
        return [
          name,
          issueIds,
          incompleteIds,
          dependencies,
          depNames,
          depFrames,
        ];
      },
      [sectionId, initialBlocked]
    ),
    shallow
  );

  return (
    <Box direction="column" width="100%">
      {dependencies.map((dep, i) => (
        <Box
          key={dep}
          direction="row"
          align="middle"
          justify="between"
          margin={{ bottom: "xsmall" }}
          style={{
            padding: 10,
            borderRadius: 4,
            border: `1px solid ${frameStyles.colors[depFrames[i]]}`,
            backgroundColor: `${frameStyles.colors[depFrames[i]]}44`,
          }}
        >
          <span style={{ marginRight: 20 }}>
            Resolve <b>{depNames[i]}</b> before continuing
          </span>
          {frame !== depFrames[i] && (
            <FrameButton
              frame={depFrames[i]}
              active={false}
              text={`Go to ${
                frameNames[sectionFrame(dependencies[i], frames)]
              }`}
              onClick={() => setFrame(depFrames[i])}
            />
          )}
        </Box>
      ))}

      <Collapse
        disabled={blocked}
        defaultOpen={false}
        header={name}
        //   style={{ marginBottom: 5 }}
        extra={
          <>
            {incompleteIds.length > 0 ? (
              <Chip size="small" color="primary" label={incompleteIds.length} />
            ) : (
              <Avatar
                sx={{
                  width: 23,
                  height: 23,
                  backgroundColor: theme.palette.quiet.main,
                  boxShadow: `0px 0px 2px 2px ${theme.palette.primary.main}`
                }}
                color="primary"
              >
                <FiCheck style={{ width: 15, height: 15, color: theme.palette.primary.main }} />
              </Avatar>
            )}
            {/* <Switch
              color="primary"
              checked={complete || incompleteIds.length === 0}
              onChange={() => {}}
            /> */}
          </>
        }
      >
        {issueIds.length > 0 ? (
          <>
            {issueIds.map((issue) => (
              <ReviewIssue key={issue} issueId={issue} />
            ))}
          </>
        ) : (
          <Typography sx={{ textAlign: "center" }}>No Issues</Typography>
        )}
      </Collapse>
    </Box>
  );
});
