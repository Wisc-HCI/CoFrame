import React, { useCallback, memo } from "react";
import { Group } from "@visx/group";
import { AxisBottom, AxisLeft } from "@visx/axis";
import { scaleBand, scaleLinear, scaleOrdinal } from "@visx/scale";
import { withTooltip, TooltipWithBounds, defaultStyles } from "@visx/tooltip";
import { LinearGradient } from "@visx/gradient";
import { localPoint } from "@visx/event";
import { LegendOrdinal } from "@visx/legend";
import useStore from "../stores/Store";
import { uniq } from "lodash";
import { Box, Button, Notification, Text } from "grommet";
import { STATUS, STEP_TYPE, TIMELINE_TYPES } from "../stores/Constants";
import { useSpring, animated } from "@react-spring/web";
import { useTime } from "./useTime";

export const background = "#eaedff";
const defaultMargin = { top: 40, left: 80, right: 40, bottom: 50 };

const tooltipStyles = {
  ...defaultStyles,
  minWidth: 60,
  backgroundColor: "rgba(0,0,0,0.9)",
  color: "white",
  padding: 5,
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
  // bounds

  // const focus = useStore((state) => state.focus);
  //   const [pause, play, reset] = useStore((state) => [
  //     state.pause,
  //     state.play,
  //     state.reset,
  //   ]);
  const [focusSteps, errorType] = useStore((state) => {
    let steps = [];
    let errorType = null;
    console.log("STUFF",{programData:state.programData,focus:state.focus,programSpec:state.programSpec})
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
  // console.log('focusSteps', focusSteps)

  const clearFocus = useStore((state) => state.clearFocus);
  const primaryColor = useStore((state) => state.primaryColor);
  const [stepData, trackTypes] = useStore((state) => {
    let robot = Object.values(state.programData).filter(
      (v) => v.type === "robotAgentType"
    )[0];
    let trackTypes = {
      [robot.id]: {
        label: robot.name,
        color: state.programSpec.objectTypes.delayType.instanceBlock.color,
      },
    };
    let stepData = [];
    let openSteps = {};
    [...focusSteps, { type: null, time: Number.INFINITY }].forEach(
      (step, i) => {
        // Package up any statuses if they no longer match the current time;
        if (
          [STEP_TYPE.ACTION_START, STEP_TYPE.PROCESS_START].includes(step.type)
        ) {
          const track =
            step.type === STEP_TYPE.ACTION_START
              ? robot.id
              : step.data.machine
              ? step.data.machine
              : step.data.process;
          openSteps[step.source] = {
            track: track,
            event: step.type === STEP_TYPE.ACTION_START ? "action" : "process",
            label: state.programData[step.source]
              ? state.programData[step.source].name
              : "Updating...",
            start: step.time,
            end: null,
          };
        } else if (
          [STEP_TYPE.ACTION_END, STEP_TYPE.PROCESS_END].includes(step.type)
        ) {
          openSteps[step.source].end = step.time;
          stepData.push(openSteps[step.source]);
          delete openSteps[step.source];
        } else if (
          [STEP_TYPE.SPAWN_ITEM, STEP_TYPE.DESTROY_ITEM].includes(step.type)
        ) {
          const track = step.data.machine
            ? step.data.machine
            : step.data.process;
          if (!trackTypes[track]) {
            let objType = state.programData[track].type;
            trackTypes[track] = {
              label: state.programData[track].name,
              color:
                state.programSpec.objectTypes[objType].referenceBlock.color,
            };
          }
          stepData.push({
            time: step.time,
            track,
            event: "things",
            label: `${
              state.programData[step.data.thing]
                ? state.programData[step.data.thing].name
                : "Thing"
            } ${step.type === STEP_TYPE.SPAWN_ITEM ? "Spawned" : "Consumed"}`,
          });
        } else if (step.type === STEP_TYPE.LANDMARK) {
          stepData.push({
            time: step.time,
            track: robot.id,
            event: "action",
            label: step.data.label,
          });
        }
      }
    );
    return [stepData, trackTypes];
  });

  // console.warn("STEP DATA", stepData);

  const eventTypes = useStore((state) => ({
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
  }));

  if (!visible) {
    return null
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
      stepData={stepData}
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
    stepData,
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
    primaryColor,
  }) => {
    const xMax = width - margin.left - margin.right;
    const yMax = height - margin.top - margin.bottom;

    const lastEnd =
      Math.max(...stepData.map((e) => (e.time ? e.time : e.end))) + 500;
    
    // console.log(stepData.map((e) => (e.time ? e.time : e.end)))

    const yScale = scaleBand({
      domain: stepData.map((e) => e.track),
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
      (0.8 * yMax) / uniq(stepData.map((e) => e.track)).length
    );

    xScale.rangeRound([0, xMax]);
    yScale.rangeRound([0, yMax]);

    const [pause, play, reset] = useStore((state) => [
      state.pause,
      state.play,
      state.reset,
    ]);

    const handleTooltip = useCallback(
      (event, paused) => {
        const { x, y } = localPoint(event) || { x: -10, y: -10 };
        const x0 = xScale.invert(x - defaultMargin.left);
        const d = stepData
          .filter((e) => {
            if (e.time) {
              // console.log({e, time: e.time, x0, pointSensitivity})
              return Math.abs(e.time - x0) < pointSensitivity;
            } else {
              return e.start <= x0 && e.end >= x0;
            }
          })
          .map((e) => ({ ...e, progress: x0 - e.start }));
        if (paused) {
            pause();
            reset(x0/1000);
            showTooltip({
                tooltipData: d,
                tooltipLeft: x - defaultMargin.left,
                tooltipTop: y - defaultMargin.top,
              });
        } else {
            play();
            hideTooltip();
        }
        
      },
      [showTooltip, xScale, stepData, hideTooltip, pause, play, reset]
    );

    // const clock = useStore((state) => state.clock);
    

    // const [visualTime, setVisualTime] = useState(0);

    // useEffect(() => {
    //   const interval = setInterval(() => {
    //     const time = (clock.getElapsed() * 1000) % lastEnd;
    //     setVisualTime(time);
    //   }, lastEnd / 150);
    //   return () => clearInterval(interval);
    // });

    return (
      <div>
        <svg width={width} height={height}>
          <Group
            top={margin.top}
            left={margin.left}
            onTouchStart={(e)=>{
                handleTooltip(e,true);
            }}
            onTouchMove={(e)=>{
                handleTooltip(e,true)
            }}
            onMouseMove={(e)=>{
                handleTooltip(e,true)
            }}
            onMouseLeave={(e) => {
                handleTooltip(e,false)
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

            {stepData
              .filter((e) => e.time === undefined)
              .map((entry, i) => {
                // console.log({track:entry.track,yscale:yScale(entry.track)})
                return (
                  <g key={`${i}block`}>
                    <rect
                      rx={5}
                      x={xScale(entry.start)}
                      y={yScale(entry.track)}
                      width={xScale(entry.end - entry.start)}
                      height={barHeight}
                      fill={colorScale(entry.event)}
                      fillOpacity={0.6}
                      stroke={colorScale(entry.event)}
                      strokeOpacity={1}
                      strokeWidth={1}
                    />
                    <text
                      fontSize={12}
                      fill="white"
                      x={xScale(entry.start) + 20}
                      y={yScale(entry.track) + 0.6 * barHeight}
                    >
                      {entry.label}
                    </text>
                  </g>
                );
              })}
            {stepData
              .filter((e) => e.time !== undefined)
              .map((entry, i) => {
                // console.log({ cx: xScale(entry.time), cy: yScale(entry.track) + barHeight / 2, entry })
                return (
                  <g key={`${i}spot`}>
                    <circle
                      cx={xScale(entry.time)}
                      cy={yScale(entry.track) + barHeight / 2}
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
                      cy={yScale(entry.track) + barHeight / 2}
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
                width: defaultMargin.left - 10,
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
              })}
            />
            {lastEnd && yMax && <CurrentTimeIndicator xScale={xScale} lastEnd={lastEnd} yMax={yMax}/>}
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
            fontSize: "14px",
          }}
        >
          <LegendOrdinal
            scale={colorScale}
            direction="row"
            labelMargin="0 15px 0 0"
            labelFormat={(l) => capitalize(l)}
          />
        </div>
        {tooltipOpen && tooltipData && (
          <TooltipWithBounds
            top={tooltipTop}
            left={tooltipLeft-20}
            style={tooltipStyles}
          >
            <Box gap="xsmall">
              {tooltipData.length > 0 &&
                tooltipData.map((e, i) => (
                  <div key={i}>
                    <Box
                      direction="row"
                      alignContent="center"
                      align="center"
                      justify="start"
                      gap="xsmall"
                    >
                      {e.time !== undefined && (
                        <div
                          style={{
                            borderRadius: 100,
                            width: 7,
                            height: 7,
                            backgroundColor: colorScale(e.event),
                            boxShadow: "0 0 0 2px white",
                          }}
                        ></div>
                      )}
                      <Text
                        color={colorScale(e.event)}
                        size={e.time === undefined ? "medium" : "small"}
                      >
                        {e.label}
                      </Text>
                    </Box>

                    {e.time === undefined && (
                      <div>
                        {" "}
                        {round(e.progress / 1000)} /{" "}
                        {round((e.end - e.start) / 1000)} sec
                      </div>
                    )}
                    {e.time !== undefined && (
                      <div>@ {round(e.time / 1000)} sec</div>
                    )}
                  </div>
                ))}
              {tooltipData.length === 0 && (
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
          </TooltipWithBounds>
        )}
      </div>
    );
  }
);

const CurrentTimeIndicator = memo(({xScale, lastEnd, yMax}) =>  {
    // const clock = useStore((state) => state.clock);
    
    // const [visualTime, setVisualTime] = useState(0);
    
    const visualTime = useTime(lastEnd);
    const x = xScale(visualTime);

    const timeIndicatorLineStyle = useSpring({
      x1: x ? x : 0,
      x2: x ? x : 0,
      y1: 0,
      y2: yMax,
      config: {mass:0.25, tension:250, friction:10},
    });

    
    return (
      <animated.line
        stroke='lightgrey'
        strokeWidth={2}
        strokeDasharray="5,2"
        // x1={x}
        // x2={x}
        // y1={0}
        // y2={yMax}
        {...timeIndicatorLineStyle}
      />
    )
})

export default TimelineGraph