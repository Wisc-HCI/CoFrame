import React, { useCallback, memo } from "react";
import { Group } from "@visx/group";
import { AxisBottom, AxisLeft } from "@visx/axis";
import { scaleBand, scaleLinear, scaleOrdinal } from "@visx/scale";
import { withTooltip, Tooltip, defaultStyles } from "@visx/tooltip";
import { LinearGradient } from "@visx/gradient";
import { localPoint } from "@visx/event";
import { LegendOrdinal } from "@visx/legend";
import useStore from "../stores/Store";
import { Box, Button, Notification, Text } from "grommet";
import { STATUS, TIMELINE_TYPES } from "../stores/Constants";
import { useSpring, animated } from "@react-spring/web";
import { useTime } from "./useTime";
import { stepDataToBlocksAndTracks } from "../helpers/graphs";

const defaultMargin = { top: 40, left: 80, right: 40, bottom: 25 };

const tooltipStyles = {
  ...defaultStyles,
  minWidth: 60,
  backgroundColor: "rgba(0,0,0,0.9)",
  color: "white",
  padding: 5
};

const pointSensitivity = 500;

const round = (num) => Math.round(num * 10) / 10;

const capitalize = (row) => row.charAt(0).toUpperCase() + row.slice(1);
// const formatCol = (row) => round(row / 1000);

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

const TimelineGraph = ({ width, height, margin = defaultMargin, visible }) => {
  const [focusSteps, errorType] = useStore((state) => {
    let steps = [];
    let errorType = null;
    console.log("STUFF", {
      programData: state.programData,
      focus: state.focus,
      programSpec: state.programSpec
    });
    state.focus.some((f) => {
      const entry = state.programData[f];
      if (
        [STATUS.VALID, STATUS.PENDING, STATUS.WARN].includes(
          entry?.properties?.status
        ) &&
        TIMELINE_TYPES.includes(state.programData[f].type)
      ) {
        if (Object.keys(entry.properties?.compiled).length === 1) {
          steps =
            entry.properties.compiled[
              Object.keys(entry.properties?.compiled)[0]
            ]?.steps;
          return true;
        } else {
          errorType = "traces";
          return false;
        }
      } else {
        errorType = "invalid";
        return false;
      }
    });
    return [steps, errorType];
  });
  console.log("focusSteps", focusSteps);

  const clearFocus = useStore((state) => state.clearFocus);
  const primaryColor = useStore((state) => state.primaryColor);
  const [blockData, eventData, trackTypes] = useStore((state) => {
    let robot = Object.values(state.programData).filter(
      (v) => v.type === "robotAgentType"
    )[0];
    let trackTypes = {
      [robot.id]: {
        label: robot.name,
        color: state.programSpec.objectTypes.delayType.instanceBlock.color
      }
    };
    return stepDataToBlocksAndTracks(
      state.programData,
      state.programSpec,
      focusSteps,
      trackTypes,
      robot
    );
  });

  // console.warn("STEP DATA", blockData);

  const eventTypes = useStore((state) => ({
    action: {
      label: "Robot Action",
      color: state.programSpec.objectTypes.delayType.instanceBlock.color
    },
    process: {
      label: "Process",
      color: state.programSpec.objectTypes.processType.referenceBlock.color
    },
    machines: {
      label: "Machines",
      color: state.programSpec.objectTypes.machineType.referenceBlock.color
    },
    things: {
      label: "Things",
      color: state.programSpec.objectTypes.thingType.referenceBlock.color
    }
  }));

  if (!visible) {
    return null;
  }
  if (errorType) {
    return (
      <Box fill justifyContent="center" alignContent="center" width="100%">
        <Notification
          onClose={() => {}}
          status="warning"
          title={
            errorType === "traces"
              ? "No single trace is available to display"
              : "Selected action contains errors"
          }
          message={
            errorType === "traces"
              ? "This is usually because you are attempting to visualize an action in a skill that is used multiple times. To visualize, you will need to visualize the skill-call instead"
              : "You likely have not parameterized all fields correctly, or are missing critical values. Consult the review panel for more suggestions."
          }
          toast
        />
        <Box
          height="100%"
          alignSelf="center"
          gap="xsmall"
          direction="column"
          justify="around"
          pad="medium"
        >
          <Text size="large">
            <i>Nothing to display</i>
          </Text>
          <Button label="Close" onClick={clearFocus} />
        </Box>
      </Box>
    );
  }

  return width < 10 && height < 40 ? null : (
    <InnerGraph
      width={width}
      height={height}
      margin={margin}
      blockData={blockData}
      eventData={eventData}
      eventTypes={eventTypes}
      trackTypes={trackTypes}
      primaryColor={primaryColor}
    />
  );
};

const InnerGraph = withTooltip(
  ({
    width,
    height,
    blockData,
    eventData,
    eventTypes,
    trackTypes,
    events = false,
    margin = defaultMargin,
    tooltipOpen,
    tooltipLeft,
    tooltipTop,
    tooltipData,
    hideTooltip,
    showTooltip,
    primaryColor
  }) => {
    const xMax = width - margin.left - margin.right;
    const yMax = height - margin.top - margin.bottom;

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
      domain: Object.keys(trackTypes),
      padding: 0.2
    });
    const colorScale = scaleOrdinal({
      domain: Object.keys(eventTypes),
      range: Object.keys(eventTypes).map((e) => eventTypes[e].color)
    });

    // console.log('lastEnd', lastEnd)
    const xScale = scaleLinear({
      domain: [0, lastEnd],
      nice: true
    });

    const barHeight = Math.max(
      0,
      (0.8 * yMax) / Object.keys(trackTypes).length
    );

    xScale.rangeRound([0, xMax]);
    yScale.rangeRound([0, yMax]);

    const [pause, play, reset] = useStore((state) => [
      state.pause,
      state.play,
      state.reset
    ]);

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
          )
        };
        if (paused) {
          pause();
          reset(x0 / 1000);
          showTooltip({
            tooltipData: d,
            tooltipLeft: x - defaultMargin.left,
            tooltipRight: x + defaultMargin.right,
            tooltipTop: y - defaultMargin.top
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
        reset
      ]
    );

    return (
      <div>
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

            {blockData.map((entry, i) => {
              // console.log({track:entry.track,yscale:yScale(entry.track)})
              const width = xScale(entry.end - entry.start);
              console.log(width);
              return (
                <g key={`${i}block`}>
                  <rect
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
                  {width > 100 && (
                    <text
                      fontSize={12}
                      fill="white"
                      x={xScale(entry.start) + 20}
                      y={yScale(entry.track) + 0.6 * barHeight}
                    >
                      {entry.label}
                    </text>
                  )}
                </g>
              );
            })}
            {eventData.map((entry, i) => {
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
              tickFormat={(row) => trackTypes[row].label}
              stroke={axisColor}
              tickStroke={axisColor}
              tickLabelProps={() => ({
                fill: axisColor,
                fontSize: 11,
                textAnchor: "end",
                dy: "0.33em",
                width: defaultMargin.left - 10
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
                textAnchor: "middle"
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
                {/* <Line
                  from={{ x: tooltipLeft, y: 0 }}
                  to={{ x: tooltipLeft, y: yMax }}
                  stroke={"lightgrey"}
                  strokeWidth={2}
                  pointerEvents="none"
                  strokeDasharray="5,2"
                /> */}

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
        <div
          style={{
            position: "absolute",
            top: margin.top / 2 - 10,
            width: "100%",
            display: "flex",
            justifyContent: "center",
            fontSize: "14px"
          }}
        >
          <LegendOrdinal
            scale={colorScale}
            direction="row"
            shape="circle"
            labelMargin="0 15px 0 0"
            legendLabelProps={{ style: { color: "white", paddingRight: 8 } }}
            labelFormat={(l) => capitalize(l)}
          />
        </div>
        {tooltipOpen && tooltipData && (
          <Tooltip top={tooltipTop} left={tooltipLeft} style={tooltipStyles}>
            <Box gap="xsmall">
              {tooltipData.blocks.length > 0 ||
              tooltipData.events.length > 0 ? (
                <>
                  {tooltipData.blocks.map((e, i) => (
                    <div key={`${i}b`}>
                      <Box
                        direction="row"
                        alignContent="center"
                        align="center"
                        justify="start"
                        gap="xsmall"
                      >
                        <Text color={colorScale(e.event)} size={"medium"}>
                          {e.label}
                        </Text>
                      </Box>

                      <div>
                        {" "}
                        {round(e.progress / 1000)} /{" "}
                        {round((e.end - e.start) / 1000)} sec
                      </div>
                    </div>
                  ))}
                  {tooltipData.events.map((e, i) => (
                    <div key={`${i}e`}>
                      <Box
                        direction="row"
                        alignContent="center"
                        align="center"
                        justify="start"
                        gap="xsmall"
                      >
                        <div
                          style={{
                            borderRadius: 100,
                            width: 7,
                            height: 7,
                            backgroundColor: colorScale(e.event),
                            boxShadow: "0 0 0 2px white"
                          }}
                        />
                        <Text color={colorScale(e.event)} size={"small"}>
                          {e.label}
                        </Text>
                      </Box>
                      <div>@ {round(e.time / 1000)} sec</div>
                    </div>
                  ))}
                </>
              ) : (
                <strong style={{ color: primaryColor }}>No actions</strong>
              )}
            </Box>

            {/* <div style={{ color: colorScale(tooltipData.event) }}>
                                <strong>{tooltipData.label}</strong>
                            </div>
                            <div>{tooltipData.end - tooltipData.start} sec</div>
                            <div>
                                <small>{capitalize(tooltipData.group)}</small>
                            </div> */}
          </Tooltip>
        )}
      </div>
    );
  }
);

const CurrentTimeIndicator = memo(({ xScale, lastEnd, yMax }) => {
  // const clock = useStore((state) => state.clock);

  // const [visualTime, setVisualTime] = useState(0);

  const visualTime = useTime(lastEnd);
  const x = xScale(visualTime);

  const timeIndicatorLineStyle = useSpring({
    x1: x ? x : 0,
    x2: x ? x : 0,
    y1: 0,
    y2: yMax,
    config: { mass: 0.25, tension: 250, friction: 10 }
  });

  return (
    <animated.line
      stroke="lightgrey"
      strokeWidth={2}
      strokeDasharray="5,2"
      // x1={x}
      // x2={x}
      // y1={0}
      // y2={yMax}
      {...timeIndicatorLineStyle}
    />
  );
});

export default TimelineGraph;