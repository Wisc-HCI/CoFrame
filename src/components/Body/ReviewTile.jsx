import React from "react";
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
  Stack,
  Paper,
  Badge,
  Typography
} from "@mui/material";

const isComplete = (state, sectionId) =>
  state.sections[sectionId].issues
    .map((issueId) => state.issues[issueId])
    .filter((issue) => !issue.complete).length === 0;
const isBlocked = (state, sectionId) =>
  state.sections[sectionId].dependencies.filter(
    (dep) => !isComplete(state, dep)
  ).length > 0;

export const ReviewTile = ({drawerOpen}) => {
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
      <Stack direction="row" style={{justifyContent:'space-between',alignContent:'center',padding:5}}>
        <Typography component='span'>Review</Typography>
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
            disabled={isProcessing}
          >
            Refresh
          </Button>
        </Badge>
      </Stack>
      <ScrollRegion vertical height={`calc(${bounds.height - 105}px - ${drawerOpen ? "20vh" : "0vh"})`}>
        <Stack
          direction="column"
          style={{ width: "calc(100% - 6px)" }}
          round="small"
          spacing={0.5}
        >
          {Object.values(frames)[frameIdx].sections.map((section, idx) => (
            <ReviewSection
              key={section}
              sectionId={section}
              blocked={idx >= blockages[frameIdx]}
              initialBlocked={idx === blockages[frameIdx]}
            />
          ))}
        </Stack>
      </ScrollRegion>
    </Paper>
  );
};
