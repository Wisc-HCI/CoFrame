import React, { useCallback } from "react";
import { Box } from "grommet";
import { ReviewIssue } from "./ReviewIssue";
import useStore from "../../stores/Store";
import { FrameButton } from "../FrameButton";
import frameStyles from "../../frameStyles";
import Collapse from "../Elements/Collapse";
// import { Switch } from "../Elements/Switch";
// import { FancyText } from "../Elements/FancyText";
import Switch from '@mui/material/Switch';
import FormControlLabel from '@mui/material/FormControlLabel';

// import './custom.css'

const isComplete = (state, sectionId) =>
  state.sections[sectionId].issues
    .map((issueId) => state.issues[issueId])
    .filter((issue) => !issue.complete).length === 0;

export function ReviewSection({ sectionId, blocked, initialBlocked }) {
  const [frame, setFrame] = useStore((state) => [
    state.frame,
    state.setFrame
  ]);

  const [frames, frameNames] = useStore((state) => [state.frames, state.frameNames]);


  const sectionFrame = (sectionId) =>
    Object.values(frames).filter((filter) => filter.sections.indexOf(sectionId) > -1)[0].key;


  const [name, issueIds, complete, dependencies, depNames, depFrames] =
    useStore(
      useCallback(
        (state) => {
          const name = state.sections[sectionId].name;
          const issueIds = state.sections[sectionId].issues;
          const complete = isComplete(state, sectionId);
          const dependencies = initialBlocked
            ? state.sections[sectionId].dependencies.filter(
                (dep) => !isComplete(state, dep)
              )
            : [];
          const depNames = dependencies.map((dep) => state.sections[dep].name);
          const depFrames = dependencies.map((dep) => sectionFrame(dep));
          return [name, issueIds, complete, dependencies, depNames, depFrames];
        },
        [sectionId, initialBlocked]
      )
    );

  return (
    
      <Box direction="column" width='100%' >
        {dependencies.map((dep, i) => (
          <Box
            key={dep}
            direction="row"
            align="middle"
            justify="between"
            margin={{bottom:'xsmall'}}
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
                text={`Go to ${frameNames[sectionFrame(dependencies[i])]}`}
                onClick={() => setFrame(depFrames[i])}
              />
            )}
          </Box>
        ))}

        <Collapse
          openable={!blocked}
          borderWidth={3}
          internalPaddingWidth={5}
          header={
            <Box direction="row" pad="10pt" flex>
              {name}
            </Box>
          }
          //   style={{ marginBottom: 5 }}
          extra={
            <FormControlLabel
            value={complete}
            control={
              <Switch
                color="primaryColor"
                disabled
                value={complete}
                onChange={()=>{}}
              />
            }
            label={complete ? 'Done!' : ""}
            labelPlacement="start"
          />
            // <Switch
            //   label={complete ? <FancyText css={{ color: primaryColor, fontSize:10 }}>Done!</FancyText> : null}
            //   disabled
            //   value={complete}
            //   highlightColor={primaryColor}
            // />
          }
        >
          {issueIds.length > 0 ? (
            <Box direction="column" gap="xsmall">
              {issueIds.map((issue) => (
                <ReviewIssue key={issue} issueId={issue} />
              ))}
            </Box>
          ) : null}
        </Collapse>
      </Box>
  );
}
