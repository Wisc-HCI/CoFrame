import React from "react";
import { FiAlertCircle, FiCheckCircle, FiRefreshCcw, FiRefreshCw } from "react-icons/fi";
import useStore from "../../stores/Store";
import { shallow } from "zustand/shallow";
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
  Typography,
  IconButton,
  Tooltip,
} from "@mui/material";
import { memo } from "react";
import { ExpandCarrot } from "../Elements/ExpandCarrot";
import frameStyles from "../../frameStyles";

const isComplete = (state, sectionId) =>
  state.sections[sectionId].issues
    .map((issueId) => state.issues[issueId])
    .filter((issue) => !issue.complete).length === 0;
const isBlocked = (state, sectionId) =>
  state.sections[sectionId].dependencies.filter(
    (dep) => !isComplete(state, dep)
  ).length > 0;

export const ReviewTile = memo(({ drawerOpen }) => {
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

  const [reviewExpanded, setReviewExpanded] = useStore(
    (state) => [state.reviewExpanded, state.setReviewExpanded],
    shallow
  );

  return (
    <Paper
      ref={ref}
      sx={{
        width: reviewExpanded ? 300 : 50,
        borderRadius: 0,
        padding: "5px",
        backgroundColor: "black",
      }}
      elevation={0}
    >
      <Stack
        direction={reviewExpanded ? "row" : "column"}
        alignItems={reviewExpanded ? 'flex-start' : "center"}
        justifyContent="center"
        style={{marginBottom: reviewExpanded ? 5 : 0}}
      >
        <span>
        <IconButton
          onClick={() => {
            setReviewExpanded(!reviewExpanded);
          }}
        >
          <ExpandCarrot expanded={reviewExpanded} flip fontSize={20} />
        </IconButton>
        </span>
        <FrameTabBar
          active={frameId}
          onChange={setFrame}
          width={reviewExpanded ? 210 : 46}
          height={reviewExpanded ? 60 : 210}
          backgroundColor="transparent"
          vertical={!reviewExpanded}
        />
        {reviewExpanded && (
          <Tooltip
            arrow
            title="Refresh Feedback"
            color={frameId}
          >
            <span>
            <IconButton
              // style={{ width: 46, height: 46 }}
              color="primaryColor"
              variant="outlined"
              onClick={refresh}
              disabled={isProcessing || reviewableChanges === 0}
            >
              <FiRefreshCcw />
            </IconButton>
            </span>
          </Tooltip>
        )}
      </Stack>

      {reviewExpanded && (
        <>
          <ScrollRegion
            vertical
            height={`calc(${bounds.height - 75}px - ${
              drawerOpen ? "20vh" : "0vh"
            })`}
          >
            <Stack
              direction="column"
              style={{ width: "calc(100% - 1px)" }}
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
        </>
      )}
    </Paper>
  );
});
