import React, { useCallback, memo, useState } from "react";
import { Group } from "@visx/group";
import { AxisBottom, AxisLeft } from "@visx/axis";
import { scaleBand, scaleLinear, scaleOrdinal } from "@visx/scale";
import { withTooltip } from "@visx/tooltip";
import { LinearGradient } from "@visx/gradient";
import { localPoint } from "@visx/event";
import { Text } from "@visx/text";
// import { LegendOrdinal } from "@visx/legend";
import useStore from "../stores/Store";
// import { Box, Text } from "grommet";
// import { useSpring, animated } from "@react-spring/web";
import { motion } from "framer-motion";
import { useTime } from "./useTime";
import {
  collapseBands,
  stepDataToBlocksAndTracks,
  getMaxForAllSeries,
  getMinForAllSeries,
  collapseSeries,
  smoothInterpolateScalar,
  getBlocks,
} from "../helpers/graphs";
import shallow from "zustand/shallow";
import styled from "@emotion/styled";
import { range } from "lodash";
import { Stack, Typography } from "@mui/material";
import { DarkTooltip } from "./Elements/DarkTooltip";

const defaultMargin = { top: 40, left: 80, right: 40, bottom: 25 };

const Selector = styled.div`
  border-radius: 100px;
  width: 20px;
  height: 20px;
  cursor: pointer;
  background-color: ${(props) => props.color};
  box-shadow: ${(props) => (props.selected ? "0px 0px 2px 2px white" : null)};
  &:hover {
    opacity: 0.7;
  }
`;

const pointSensitivity = 500;

const round = (num) => Math.round(num * 10) / 10;

function formatTime(ms) {
  const seconds = ms / 1000;
  const h = Math.floor(seconds / 3600);
  const m = Math.floor((seconds % 3600) / 60);
  const s = Math.round(seconds % 60);
  return [h, m > 9 ? m : h ? "0" + m : m || "0", s > 9 ? s : "0" + s]
    .filter(Boolean)
    .join(":");
}

const axisColor = "white";

const TimelineGraph = memo(({
  width,
  height,
  margin = defaultMargin,
  focusSteps,
  issue,
}) => {
  // console.log("focusSteps", focusSteps);
  const primaryColor = useStore((state) => state.primaryColor);
  const [blockData, eventData, trackTypes] = useStore((state) => {
    let robot = Object.values(state.programData).filter(
      (v) => v.type === "robotAgentType"
    )[0];
    let trackTypes = {
      [robot.id]: {
        label: robot.name,
        color: state.programSpec.objectTypes.delayType.instanceBlock.color,
      },
    };
    return stepDataToBlocksAndTracks(
      state.programData,
      state.programSpec,
      focusSteps,
      trackTypes,
      robot
    );
  }, shallow);

  // console.warn("STEP DATA", blockData);

  const eventTypes = useStore(
    (state) => ({
      action: {
        label: "Robot Action",
        color: state.programSpec.objectTypes.delayType.instanceBlock.color,
      },
      process: {
        label: "Process",
        color: state.programSpec.objectTypes.processType.referenceBlock.color,
      },
      machines: {
        label: "Machines",
        color: state.programSpec.objectTypes.machineType.referenceBlock.color,
      },
      things: {
        label: "Things",
        color: state.programSpec.objectTypes.thingType.referenceBlock.color,
      },
    }),
    shallow
  );

  // console.log(issueGraphContent);

  return width < 10 && height < 40 ? null : (
    <InnerGraph
      width={width}
      height={height}
      margin={margin}
      blockData={blockData}
      eventData={eventData}
      issueData={issue}
      eventTypes={eventTypes}
      trackTypes={trackTypes}
      primaryColor={primaryColor}
    />
  );
});

const InnerGraph = withTooltip(
  memo(({
    width,
    height,
    blockData,
    eventData,
    issueData,
    eventTypes,
    trackTypes,
    margin = defaultMargin,
    tooltipOpen,
    tooltipLeft,
    tooltipTop,
    tooltipData,
    hideTooltip,
    showTooltip,
    primaryColor,
  }) => {
    const xMax = width - margin.left - margin.right;
    const yMax = height - margin.top - margin.bottom;

    const [expanded, setExpanded] = useState([Object.keys(eventTypes)[0]]);

    const toggleExpanded = (eventType) => {
      if (expanded.includes(eventType)) {
        setExpanded(expanded.filter((e) => e !== eventType));
      } else {
        setExpanded([...expanded, eventType]);
      }
    };

    const [filteredBlockData, filteredEventData, filteredTrackTypes] =
      collapseBands(blockData, eventData, trackTypes, expanded);

    const lastEnd = Math.max(
      Math.max(
        ...blockData.map((e) => e.end),
        ...eventData.map((e) => e.time)
      ) + 500,
      1000
    );

    // console.log(blockData.map((e) => (e.time ? e.time : e.end)))
    // console.log(Object.keys(trackTypes))
    const yScale = scaleBand({
      domain: Object.keys(filteredTrackTypes),
      padding: 0.2,
    });
    const colorScale = scaleOrdinal({
      domain: Object.keys(eventTypes),
      range: Object.keys(eventTypes).map((e) => eventTypes[e].color),
    });

    // console.log('lastEnd', lastEnd)
    const xScale = scaleLinear({
      domain: [0, lastEnd],
      nice: true,
    });

    const barHeight = Math.max(
      0,
      (0.8 * yMax) / Object.keys(filteredTrackTypes).length
    );

    xScale.rangeRound([0, xMax]);
    yScale.rangeRound([0, yMax]);

    const [pause, play, reset] = useStore(
      (state) => [state.pause, state.play, state.reset],
      shallow
    );

    const issueTimeseriesData = issueData?.series || [];
    // const issueXAxisLabel = issueData?.xAxisLabel || "";
    // const issueYAxisLabel = issueData?.yAxisLabel || "";
    const issueThresholds = issueData?.thresholds || [];
    // const issueUnits = issueData?.units || "";
    // const issueDecimals = issueData?.decimals || 0;
    const issueKeys = issueData
      ? Object.keys(issueTimeseriesData[0] || {}).filter((k) => k !== "x")
      : [];
    const collapsedSeries = collapseSeries(issueTimeseriesData, issueKeys);

    const xIssueMax = Math.max(...issueTimeseriesData.map((s) => s.x));
    const yIssueMax = getMaxForAllSeries(issueTimeseriesData, issueKeys);
    const xIssueMin = Math.min(...issueTimeseriesData.map((s) => s.x));
    const yIssueMin = getMinForAllSeries(issueTimeseriesData, issueKeys);

    const interp = issueData
      ? smoothInterpolateScalar(
          collapsedSeries.map((c) => c.x),
          collapsedSeries.map((c) => c.y)
        )
      : null;
    const interpSeries = issueData
      ? range(xIssueMin, xIssueMax + 1, (xIssueMax - xIssueMin) / 50).map(
          (x) => ({
            x,
            y: interp(x),
          })
        )
      : [];

    const filledThresholds = issueThresholds.map((t) => ({
      range: [
        t.range[0] === "MIN"
          ? yIssueMin
          : t.range[0] === "MAX"
          ? yIssueMax
          : t.range[0],
        t.range[1] === "MIN"
          ? yIssueMin
          : t.range[1] === "MAX"
          ? yIssueMax
          : t.range[1],
      ],
      color: t.color,
    }));

    const blocksAtThreshold = issueData
      ? filledThresholds.map((thresh) => getBlocks(interpSeries, thresh))
      : [];

    const handleTooltip = useCallback(
      (event, paused) => {
        const { x, y } = localPoint(event) || { x: -10, y: -10 };
        const x0 = xScale.invert(x - defaultMargin.left);
        const d = {
          blocks: blockData
            .filter((b) => b.start <= x0 && b.end >= x0)
            .map((b) => ({ ...b, progress: x0 - b.start })),
          events: eventData.filter((e) =>
            Math.abs(e.time - x0 < pointSensitivity)
          ),
        };
        if (paused) {
          pause();
          reset(x0 / 1000);
          showTooltip({
            tooltipData: d,
            tooltipLeft: x - defaultMargin.left,
            tooltipRight: x + defaultMargin.right,
            tooltipTop: y - defaultMargin.top,
          });
        } else {
          play();
          hideTooltip();
        }
      },
      [
        showTooltip,
        xScale,
        blockData,
        eventData,
        hideTooltip,
        pause,
        play,
        reset,
      ]
    );

    const threshBlockHeight = Math.min(barHeight / 4, 15);

    return (
      
        <div>
          <DarkTooltip title={<TooltipContent tooltipData={tooltipData} colorScale={colorScale} primaryColor={primaryColor}/>} followCursor arrow sx={{maxWidth:'none'}}>
        <svg width={width} height={height}>
          <Group
            top={margin.top}
            left={margin.left}
            onTouchStart={(e) => {
              handleTooltip(e, true);
            }}
            onTouchMove={(e) => {
              handleTooltip(e, true);
            }}
            onMouseMove={(e) => {
              handleTooltip(e, true);
            }}
            onMouseEnter={(e) => {
              handleTooltip(e, true);
            }}
            onMouseLeave={(e) => {
              handleTooltip(e, false);
            }}
          >
            <rect
              x={0}
              y={0}
              width={width - defaultMargin.left - defaultMargin.right}
              height={
                height - defaultMargin.top - defaultMargin.bottom > 0
                  ? height - defaultMargin.top - defaultMargin.bottom
                  : 0
              }
              fill="url(#area-background-gradient)"
            />
            <LinearGradient
              id="area-background-gradient"
              from="#55555555"
              to="#66666699"
            />

            {filteredBlockData.map((entry, i) => {
              // console.log({track:entry.track,yscale:yScale(entry.track)})
              const width = xScale(entry.end - entry.start);
              // console.log(width);
              return (
                <rect
                  key={`${i}block`}
                  rx={5}
                  x={xScale(entry.start)}
                  y={yScale(entry.track)}
                  width={width}
                  height={barHeight}
                  fill={colorScale(entry.event)}
                  fillOpacity={0.6}
                  stroke={colorScale(entry.event)}
                  strokeOpacity={1}
                  strokeWidth={1}
                />
              );
            })}
            {blocksAtThreshold.map((t, i) => (
              <g key={`issue-${i}`}>
                {t.map((b, j) => (
                  <rect
                    key={`${i}-${j}`}
                    x={xScale(b.x0)}
                    y={yScale(filteredBlockData[0].track) + barHeight - threshBlockHeight*1.25}
                    width={xScale(b.x1 - b.x0)}
                    height={threshBlockHeight}
                    fill={issueThresholds[i].color}
                    rx={threshBlockHeight/2}
                  />
                ))}
              </g>
            ))}
            {filteredBlockData
              .filter((entry) => xScale(entry.end - entry.start) > 100)
              .map((entry, i) => {
                // console.log({track:entry.track,yscale:yScale(entry.track)})
                return (
                  <Text
                    key={`${i}block-text`}
                    fontFamily="helvetica"
                    scaleToFit
                    textAnchor="start"
                    verticalAnchor="middle"
                    dy="-0.15em"
                    lineHeight={barHeight * 0.6}
                    width={100}
                    fill="white"
                    line="black"
                    x={xScale(entry.start) + 20}
                    y={yScale(entry.track) + 0.6 * barHeight}
                  >
                    {entry.label}
                  </Text>
                );
              })}
            {filteredEventData.map((entry, i) => {
              return (
                <g key={`${i}spot`}>
                  <circle
                    cx={xScale(entry.time)}
                    cy={yScale(entry.track) + barHeight / 2.5}
                    r={barHeight / 5}
                    fill="black"
                    fillOpacity={0.1}
                    stroke="black"
                    strokeOpacity={0.1}
                    strokeWidth={2}
                    pointerEvents="none"
                  />
                  <circle
                    cx={xScale(entry.time)}
                    cy={yScale(entry.track) + barHeight / 2.5}
                    r={barHeight / 5}
                    fill={colorScale(entry.event)}
                    stroke="white"
                    strokeWidth={2}
                    pointerEvents="none"
                  />
                </g>
              );
            })}
            <AxisLeft
              // hideAxisLine
              hideTicks
              scale={yScale}
              tickFormat={(row) => filteredTrackTypes[row].label}
              stroke={axisColor}
              tickStroke={axisColor}
              tickLabelProps={() => ({
                fill: axisColor,
                fontSize: 11,
                textAnchor: "end",
                dy: "0.33em",
                width: defaultMargin.left - 10,
                fontFamily: "helvetica",
              })}
            />
            <AxisBottom
              top={yMax}
              scale={xScale}
              tickFormat={formatTime}
              stroke={axisColor}
              tickStroke={axisColor}
              tickLabelProps={() => ({
                fill: axisColor,
                fontSize: 11,
                textAnchor: "middle",
                fontFamily: "helvetica",
              })}
            />
            {lastEnd && yMax && (
              <CurrentTimeIndicator
                xScale={xScale}
                lastEnd={lastEnd}
                yMax={yMax}
              />
            )}
            {tooltipData && (
              <g>
                <circle
                  cx={tooltipLeft}
                  cy={tooltipTop + 1}
                  r={4}
                  fill="black"
                  fillOpacity={0.1}
                  stroke="black"
                  strokeOpacity={0.1}
                  strokeWidth={2}
                  pointerEvents="none"
                />
                <circle
                  cx={tooltipLeft}
                  cy={tooltipTop}
                  r={4}
                  fill={primaryColor}
                  stroke="white"
                  strokeWidth={2}
                  pointerEvents="none"
                />
              </g>
            )}
          </Group>
        </svg>
        </DarkTooltip>
        <div
          style={{
            position: "absolute",
            top: margin.top / 2 - 10,
            width: "100%",
            display: "flex",
            justifyContent: "center",
            fontSize: "14px",
          }}
        >
          <Stack direction="row" spacing={1}>
            {Object.entries(eventTypes).map(([eventTypeKey, eventType]) => (
              <Stack
                key={eventTypeKey}
                onClick={() => toggleExpanded(eventTypeKey)}
                direction="row"
                alignItems="center"
                spacing={0.5}
              >
                <Selector
                  color={eventType.color}
                  selected={expanded.includes(eventTypeKey)}
                />
                <Typography>{eventType.label}</Typography>
              </Stack>
            ))}
          </Stack>
        </div>
      </div>
      
    );
  })
);

const CurrentTimeIndicator = memo(({ xScale, lastEnd, yMax }) => {
  // const clock = useStore((state) => state.clock);

  // const [visualTime, setVisualTime] = useState(0);

  const visualTime = useTime(lastEnd);
  // console.log(visualTime)
  const x = xScale(visualTime);

  // const timeIndicatorLineStyle = useSpring({
  //   x1: x ? x : 0,
  //   x2: x ? x : 0,
  //   y1: 0,
  //   y2: yMax,
  //   config: { mass: 0.25, tension: 250, friction: 10 },
  // });

  return (
    <motion.line
      stroke="lightgrey"
      strokeWidth={2}
      strokeDasharray="5,2"
      // x1={x}
      // x2={x}
      // y1={0}
      // y2={yMax}
      animate={{ x1: x, x2: x, y1: 0, y2: yMax }}
      // {...timeIndicatorLineStyle}
    />
  );
});

const TooltipContent = ({ tooltipData, colorScale, primaryColor }) => {
  return tooltipData ? (
    <Stack spacing={0.5}>
      {tooltipData.blocks.length > 0 || tooltipData.events.length > 0 ? (
        <>
          {tooltipData.blocks.map((e, i) => (
            <div key={`${i}b`}>
              <Stack
                direction="row"
                alignContent="center"
                align="center"
                justify="start"
                spacing={0.5}
              >
                <Typography color={colorScale(e.event)} size={"medium"}>
                  {e.label}
                </Typography>
              </Stack>

              <Typography>
                {" "}
                {round(e.progress / 1000)} / {round((e.end - e.start) / 1000)}{" "}
                sec
              </Typography>
            </div>
          ))}
          {tooltipData.events.map((e, i) => (
            <div key={`${i}e`}>
              <Stack
                direction="row"
                alignItems="center"
                align="center"
                justify="center"
                spacing={0.5}
              >
                <div
                  style={{
                    borderRadius: 100,
                    width: 7,
                    height: 7,
                    backgroundColor: colorScale(e.event),
                    boxShadow: "0 0 0 2px white",
                  }}
                />
                <Typography color={colorScale(e.event)} size={"small"}>
                  {e.label}
                </Typography>
              </Stack>
              <Typography style={{marginLeft:10}} variant='caption'>@ {round(e.time / 1000)} sec</Typography>
            </div>
          ))}
        </>
      ) : (
        <strong style={{ color: primaryColor }}>No actions</strong>
      )}
    </Stack>
  ) : null;
};

export default TimelineGraph;
