import React from "react";
import { Box } from "grommet";
import { FiRefreshCw } from "react-icons/fi";
import useStore from "../../stores/Store";
import shallow from "zustand/shallow";
import { ReviewSection } from "../Review/ReviewSection";
import useMeasure from "react-use-measure";
import { FrameTabBar } from "../FrameTabBar";
import { ScrollRegion } from "../Elements/ScrollRegion";
import { stringEquality } from "../../helpers/performance";
import {
  Button,
  BottomNavigation,
  BottomNavigationAction,
  Paper,
  Badge,
} from "@mui/material";
// import * as ScrollArea from "@radix-ui/react-scroll-area";
// import { styled } from "@stitches/react";

// const StyledScrollArea = styled(ScrollArea.Root, {
//   overflow: "hidden",
// });

// const StyledViewport = styled(ScrollArea.Viewport, {
//   width: "100%",
//   height: "100%",
//   borderRadius: "inherit",
//   padding: "4pt",
// });

// const StyledScrollbar = styled(ScrollArea.Scrollbar, {
//   display: "flex",
//   // ensures no selection
//   userSelect: "none",
//   // disable browser handling of all panning and zooming gestures on touch devices
//   touchAction: "none",
//   padding: 2,
//   background: "#55555525",
//   transition: "background 160ms ease-out",
//   "&:hover": { background: "#45454540" },
//   '&[data-orientation="vertical"]': { width: 8 },
//   '&[data-orientation="horizontal"]': {
//     flexDirection: "column",
//     height: 8,
//   },
// });

// const StyledThumb = styled(ScrollArea.Thumb, {
//   flex: 1,
//   background: "#eeeeee66",
//   borderRadius: 8,
// });

const isComplete = (state, sectionId) =>
  state.sections[sectionId].issues
    .map((issueId) => state.issues[issueId])
    .filter((issue) => !issue.complete).length === 0;
const isBlocked = (state, sectionId) =>
  state.sections[sectionId].dependencies.filter(
    (dep) => !isComplete(state, dep)
  ).length > 0;

export const ReviewTile = (_) => {
  const frames = useStore((state) => state.frames);

  const [frameId, setFrame, refresh, blockages] = useStore(
    (state) => [
      state.frame,
      state.setFrame,
      state.refresh,
      Object.values(frames).map((frameInfo) =>
        Math.min(
          ...frameInfo.sections.map((sectionId, idx) =>
            isBlocked(state, sectionId) ? idx : 100
          )
        )
      ),
    ],
    stringEquality
  );

  const isProcessing = useStore(
    (state) =>
      state.processes.planProcess !== null &&
      state.processes.planProcess !== undefined,
    shallow
  );

  const reviewableChanges = useStore(
    (state) => state.reviewableChanges,
    shallow
  );

  const frameIdx = Object.values(frames)
    .map((frame) => frame.key)
    .indexOf(frameId);

  const [ref, bounds] = useMeasure();
  //   const divStyle = useSpring({ width: "100%", config: config.stiff });

  return (
    <Paper
      ref={ref}
      sx={{
        width: 350,
        borderRadius: 0,
        padding: "5px",
        backgroundColor: "black",
      }}
      elevation={0}
    >
      <FrameTabBar
        active={frameId}
        onChange={setFrame}
        width={338}
        backgroundColor="transparent"
      />
      <Box direction="row" flex justify="between" align="center">
        <h3 style={{ margin: "10pt" }}>Review</h3>
        <Badge
          color="primary"
          badgeContent={<b>!</b>}
          invisible={reviewableChanges === 0}
          anchorOrigin={{ vertical: "top", horizontal: "left" }}
        >
          <Button
            color="primaryColor"
            variant="outlined"
            size="small"
            startIcon={<FiRefreshCw />}
            onClick={refresh}
            label="Refresh"
            disabled={isProcessing}
          >
            Refresh
          </Button>
        </Badge>
      </Box>
      <ScrollRegion vertical height={bounds.height - 125}>
        <Box
          direction="column"
          style={{ width: "calc(100% - 6px)" }}
          round="small"
          gap="xsmall"
        >
          {Object.values(frames)[frameIdx].sections.map((section, idx) => (
            <ReviewSection
              key={section}
              sectionId={section}
              blocked={idx >= blockages[frameIdx]}
              initialBlocked={idx === blockages[frameIdx]}
            />
          ))}
        </Box>
      </ScrollRegion>
    </Paper>
  );
};
