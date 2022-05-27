import React from "react";
import { Box, Button } from "grommet";
import { FiRefreshCw } from "react-icons/fi";
import useStore from "../../stores/Store";
import shallow from "zustand/shallow";
import { ReviewSection } from "../Review/ReviewSection";
import useMeasure from "react-use-measure";
import { FrameTabBar } from "../FrameTabBar";
import * as ScrollArea from "@radix-ui/react-scroll-area";
import { styled } from "@stitches/react";

const StyledScrollArea = styled(ScrollArea.Root, {
  overflow: "hidden",
});

const StyledViewport = styled(ScrollArea.Viewport, {
  width: "100%",
  height: "100%",
  borderRadius: "inherit",
  padding: "4pt",
});

const StyledScrollbar = styled(ScrollArea.Scrollbar, {
  display: "flex",
  // ensures no selection
  userSelect: "none",
  // disable browser handling of all panning and zooming gestures on touch devices
  touchAction: "none",
  padding: 2,
  background: "#55555525",
  transition: "background 160ms ease-out",
  "&:hover": { background: "#45454540" },
  '&[data-orientation="vertical"]': { width: 8 },
  '&[data-orientation="horizontal"]': {
    flexDirection: "column",
    height: 8,
  },
});

const StyledThumb = styled(ScrollArea.Thumb, {
  flex: 1,
  background: "#eeeeee66",
  borderRadius: 8,
});

const isComplete = (state, sectionId) =>
  state.sections[sectionId].issues
    .map((issueId) => state.issues[issueId])
    .filter((issue) => !issue.complete).length === 0;
const isBlocked = (state, sectionId) =>
  state.sections[sectionId].dependencies.filter(
    (dep) => !isComplete(state, dep)
  ).length > 0;

const FRAMES = [
  {
    key: "safety",
    title: "Safety Concerns",
    sections: [
      "endEffectorPoses",
      "thingMovement",
      "collisions",
      "pinchPoints",
      "occupancy",
    ],
  },
  {
    key: "quality",
    title: "Program Quality",
    sections: [
      "missingBlocks",
      "missingParameters",
      "machineLogic",
      "unusedSkills",
      "unusedFeatures",
      "emptyBlocks",
    ],
  },
  {
    key: "performance",
    title: "Robot Performance",
    sections: [
      "reachability",
      "jointSpeed",
      "endEffectorSpeed",
      "payload",
      "spaceUsage",
    ],
  },
  {
    key: "business",
    title: "Business Objectives",
    sections: ["cycleTime", "idleTime", "returnOnInvestment"],
  },
];

export const ReviewTile = (_) => {
  const [frameId, setFrame, refresh, blockages] = useStore(
    (state) => [
      state.frame,
      state.setFrame,
      state.refresh,
      FRAMES.map((frameInfo) =>
        Math.min(
          ...frameInfo.sections.map((sectionId, idx) =>
            isBlocked(state, sectionId) ? idx : 100
          )
        )
      ),
    ],
    shallow
  );

  const frameIdx = FRAMES.map((frame) => frame.key).indexOf(frameId);

  const [ref, bounds] = useMeasure();
//   const divStyle = useSpring({ width: "100%", config: config.stiff });

  return (
    <Box ref={ref} direction="column" height='100%' width="100%" pad='xsmall'>
        <FrameTabBar active={frameId} onChange={setFrame} />
        <Box direction="row" flex justify="between" align="center">
          <h3 style={{ margin: "10pt" }}>Review</h3>
          <Button
            secondary
            size="small"
            round="none"
            icon={<FiRefreshCw />}
            onClick={refresh}
            label="Refresh"
          ></Button>
        </Box>
      <StyledScrollArea css={{ height: bounds.height - 125, width: "100%"}}>
        <StyledViewport css={{padding:0}}>
          <Box direction='column' round='small'>
              {FRAMES[frameIdx].sections.map((section, idx) => (
                <Box
                  key={idx}
                  animation={{ type: "fadeIn", delay: idx * 100 }}
                  style={{ marginBottom: 5 }}
                  direction="row"
                  flex
                  height="100px"
                  round='small'
                >
                  <ReviewSection sectionId={section} blocked={idx >= blockages[frameIdx]} initialBlocked={idx === blockages[frameIdx]} />
                </Box>
              ))}
            </Box>
        </StyledViewport>
        <StyledScrollbar orientation="horizontal">
          <StyledThumb />
        </StyledScrollbar>
        <StyledScrollbar orientation="vertical">
          <StyledThumb />
        </StyledScrollbar>
        <ScrollArea.Corner />
      </StyledScrollArea>
    </Box>
  );
};
