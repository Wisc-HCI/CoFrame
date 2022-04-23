import React, { useCallback } from "react";
import { Group } from "@visx/group";
import { AxisBottom, AxisLeft } from "@visx/axis";
import { Line } from "@visx/shape";
import { scaleBand, scaleLinear, scaleOrdinal } from "@visx/scale";
import { withTooltip, Tooltip, defaultStyles } from "@visx/tooltip";
import { LinearGradient } from '@visx/gradient';
import { localPoint } from '@visx/event';
import { LegendOrdinal } from "@visx/legend";
import useStore from "../stores/Store";
import { uniq } from 'lodash';
import { Box, Button, Notification, Text } from "grommet";
import { STATUS, STEP_TYPE } from "../stores/Constants";

export const background = "#eaedff";
const defaultMargin = { top: 40, left: 80, right: 40, bottom: 50 };

const tooltipStyles = {
    ...defaultStyles,
    minWidth: 60,
    backgroundColor: "rgba(0,0,0,0.9)",
    color: "white",
    padding: 5
};

const pointSensitivity = 500;

const round = (num) => Math.round(num * 10) / 10

const formatRow = (row) => row.charAt(0).toUpperCase() + row.slice(1);
// const formatCol = (row) => round(row / 1000);

function formatTime(ms) {
    const seconds = ms/1000;
    const h = Math.floor(seconds / 3600);
    const m = Math.floor((seconds % 3600) / 60);
    const s = Math.round(seconds % 60);
    return [
      h,
      m > 9 ? m : (h ? '0' + m : m || '0'),
      s > 9 ? s : '0' + s
    ].filter(Boolean).join(':');
  }

const axisColor = 'white';

export default withTooltip(
    ({
        width,
        height,
        events = false,
        margin = defaultMargin,
        tooltipOpen,
        tooltipLeft,
        tooltipTop,
        tooltipData,
        hideTooltip,
        showTooltip
    }) => {
        // bounds
        const xMax = width - margin.left - margin.right;
        const yMax = height - margin.top - margin.bottom;

        const focus = useStore(state => state.focus);
        const [focusSteps, errorType] = useStore(state => {
            let steps = [];
            let errorType = null;
            state.focus.some(f => {
                const entry = state.programData[f];
                if ([STATUS.VALID, STATUS.PENDING, STATUS.WARN].includes(entry?.properties?.status)) {
                    if (Object.keys(entry.properties?.compiled).length === 1) {
                        steps = entry.properties.compiled[Object.keys(entry.properties?.compiled)[0]]?.steps;
                        return true
                    } else {
                        errorType = 'traces'
                        return false
                    }
                } else {
                    errorType = 'invalid'
                    return false
                }
            })
            return [steps, errorType]
        })

        console.log('focusSteps', focusSteps)

        const clearFocus = useStore(state => state.clearFocus);
        const primaryColor = useStore(state => state.primaryColor);
        const [stepData, trackTypes] = useStore(state => {
            let robot = Object.values(state.programData).filter(v => v.type === 'robotAgentType')[0];
            let trackTypes = { [robot.id]: { label: robot.name, color: state.programSpec.objectTypes.delayType.instanceBlock.color } };
            let stepData = [];
            let openSteps = {};
            [...focusSteps, { type: null, time: Number.INFINITY }].forEach((step, i) => {

                // Package up any statuses if they no longer match the current time;
                if ([STEP_TYPE.ACTION_START, STEP_TYPE.PROCESS_START].includes(step.type)) {
                    const track = step.type === STEP_TYPE.ACTION_START ? robot.id : step.data.machine ? step.data.machine : step.data.process;
                    openSteps[step.source] = {
                        track: track,
                        event: step.type == STEP_TYPE.ACTION_START ? 'action' : 'process',
                        label: state.programData[step.source] ? state.programData[step.source].name : 'Updating...',
                        start: step.time,
                        end: null
                    }
                } else if ([STEP_TYPE.ACTION_END, STEP_TYPE.PROCESS_END].includes(step.type)) {
                    openSteps[step.source].end = step.time;
                    stepData.push(openSteps[step.source]);
                    delete openSteps[step.source];
                } else if ([STEP_TYPE.SPAWN_ITEM, STEP_TYPE.DESTROY_ITEM].includes(step.type)) {
                    const track = step.data.machine ? step.data.machine : step.data.process;
                    if (!trackTypes[track]) {
                        let objType = state.programData[track].type;
                        trackTypes[track] = { label: state.programData[track].name, color: state.programSpec.objectTypes[objType].referenceBlock.color };
                    }
                    stepData.push({
                        time: step.time,
                        track,
                        event: 'things',
                        label: `${state.programData[step.data.thing] ? state.programData[step.data.thing].name : 'Thing'} ${step.type === STEP_TYPE.SPAWN_ITEM ? 'Spawned' : 'Consumed'}`
                    })
                } else if (step.type === STEP_TYPE.LANDMARK) {
                    stepData.push({
                        time: step.time,
                        track: robot.id,
                        event: 'action',
                        label: step.data.label
                    })
                }
            })
            return [stepData, trackTypes]
        });

        const eventTypes = useStore(state=>({
            action: {label: 'Robot Action', color: state.programSpec.objectTypes.delayType.instanceBlock.color},
            process: {label: 'Process', color: state.programSpec.objectTypes.processType.referenceBlock.color},
            machines: {label: 'Machines', color: state.programSpec.objectTypes.machineType.referenceBlock.color},
            things: {label: 'Things', color: state.programSpec.objectTypes.thingType.referenceBlock.color}
        }))

        // const [stepData, errorType] = useStore(state => {
        //     let activeStep = [];
        //     let groups = [];
        //     let errorType = null;
        //     state.focus.some(f => {
        //         if (state.programData[f]?.properties?.status === STATUS.VALID || state.programData[f]?.properties?.status === STATUS.PENDING) {
        //             if (Object.keys(state.programData[f].properties?.compiled).length === 1) {
        //                 let currentTime = 0;
        //                 const steps = state.programData[f].properties.compiled[Object.keys(state.programData[f].properties?.compiled)[0]]?.steps;
        //                 if (steps) {
        //                     steps.forEach((step,i) => {
        //                         if (step.stepType === STEP_TYPE.ACTION_START) {
        //                             let actionEnd = null;
        //                             steps.slice(i).some(afterStep=>{
        //                                 if (afterStep.stepType === STEP_TYPE.ACTION_END && afterStep.data.id === step.data.id) {
        //                                     actionEnd = afterStep;
        //                                     return true
        //                                 } else {
        //                                     return false
        //                                 }
        //                             })
        //                             const actionBlock = {
        //                                 group: 'Robot',
        //                                 event: 'action',
        //                                 label: state.programData[step.source] ? state.programData[step.source].name : 'Updating...',
        //                                 start: currentTime,
        //                                 end: actionEnd?actionEnd.time:0
        //                             }
        //                             currentTime = actionEnd?actionEnd.time:currentTime
        //                             activeStep.push(actionBlock)
        //                         } else if (step.stepType === STEP_TYPE.PROCESS_START) {
        //                             let processEnd = null;
        //                             steps.slice(i).some(afterStep=>{
        //                                 if (afterStep.stepType === STEP_TYPE.LANDMARK && afterStep.source === step.source) {
        //                                     processEnd = afterStep;
        //                                     return true
        //                                 } else {
        //                                     return false
        //                                 }
        //                             })
        //                             const group = step.data.machine ? state.programData[step.data.machine].name : 'Robot'
        //                             const process = state.programData[step.data.process] ? state.programData[step.data.process] : {name:'unknown'};
        //                             const processBlock = {
        //                                 group,
        //                                 event: 'process',
        //                                 label: process.name,
        //                                 start: currentTime,
        //                                 end: processEnd?processEnd.time:0
        //                             }
        //                             currentTime = processEnd?processEnd.time:currentTime
        //                             activeStep.push(processBlock)
        //                         }
        //                     })
        //                 }
        //                 return true
        //             } else if (Object.keys(state.programData[f].properties?.compiled).length > 1) {
        //                 errorType = 'traces'
        //                 return false
        //             }
        //         } else {
        //             errorType = 'invalid'
        //             return false
        //         }
        //     })
        //     return [activeStep, errorType]
        // })
        console.log({ stepData, errorType })

        const lastEnd = Math.max(...stepData.map((e) => e.time ? e.time : e.end));

        const yScale = scaleBand({
            domain: stepData.map(e=>e.track),
            padding: 0.2
        });
        const colorScale = scaleOrdinal({
            domain: Object.keys(eventTypes),
            range: Object.keys(eventTypes).map((e) => eventTypes[e].color)
        });

        console.log('lastEnd', lastEnd)
        const xScale = scaleLinear({
            domain: [0, lastEnd],
            nice: true
        });

        const barHeight = Math.max(0, 0.8 * yMax / uniq(stepData.map(e=>e.track)).length)

        xScale.rangeRound([0, xMax]);
        yScale.rangeRound([0, yMax]);

        // console.log(xScale);
        // const xMax = stepData.map(e=>e.end);

        const handleTooltip = useCallback(
            (event) => {
                const { x, y } = localPoint(event) || { x: -10, y: -10 };
                const x0 = xScale.invert(x - defaultMargin.left);
                const d = stepData
                    .filter(e => {
                        if (e.time) {
                            console.log({e, time: e.time, x0, pointSensitivity})
                            return Math.abs(e.time - x0) < pointSensitivity
                        } else {
                            return e.start <= x0 && e.end >= x0
                        }
                    })
                    .map(e => ({ ...e, progress: x0 - e.start }))
                showTooltip({
                    tooltipData: d,
                    tooltipLeft: x - defaultMargin.left,
                    tooltipTop: y - defaultMargin.top,
                });
            },
            [showTooltip, yScale, xScale, stepData],
        );

        if (errorType) {
            return (
                <Box fill justifyContent='center' alignContent='center' width='100%'>
                    <Notification
                        onClose={() => { }}
                        status='warning'
                        title={errorType === 'traces'
                            ? 'No single trace is available to display'
                            : 'Selected action contains errors'}
                        message={errorType === 'traces'
                            ? 'This is usually because you are attempting to visualize an action in a skill that is used multiple times. To visualize, you will need to visualize the skill-call instead'
                            : 'You likely have not parameterized all fields correctly, or are missing critical values. Consult the review panel for more suggestions.'}
                        toast
                    />
                    <Box height='100%' alignSelf="center" gap='xsmall' direction='column' justify='around' pad='medium'>
                        <Text size='large' >
                            <i>Nothing to display</i>
                        </Text>
                        <Button label="Close" onClick={clearFocus} />
                    </Box>

                </Box>
            )
        }


        return width < 10 && height < 40 ? null : (
            <>
                <div>
                    <svg width={width} height={height}>
                        <Group
                            top={margin.top}
                            left={margin.left}
                            onTouchStart={handleTooltip}
                            onTouchMove={handleTooltip}
                            onMouseMove={handleTooltip}
                            onMouseLeave={() => hideTooltip()}
                        >
                            <rect
                                x={0}
                                y={0}
                                width={width - defaultMargin.left - defaultMargin.right}
                                height={height - defaultMargin.top - defaultMargin.bottom > 0 ? height - defaultMargin.top - defaultMargin.bottom : 0}
                                fill="url(#area-background-gradient)"
                            />
                            <LinearGradient id="area-background-gradient" from='#55555555' to='#66666699' />

                            {stepData.filter(e => e.time === undefined).map((entry, i) => {
                                console.log({track:entry.track,yscale:yScale(entry.track)})
                                return (
                                    <g key={`${i}block`}>
                                        <rect
                                            rx={5}
                                            x={xScale(entry.start)}
                                            y={yScale(entry.track)}
                                            width={xScale(entry.end - entry.start)}
                                            height={barHeight}
                                            fill={colorScale(entry.event)}
                                        />
                                        <text
                                            fontSize={12}
                                            fill='white'
                                            x={xScale(entry.start) + 20}
                                            y={yScale(entry.track) + 0.6 * barHeight}
                                        >
                                            {entry.label}
                                        </text>
                                    </g>

                                );
                            })}
                            {stepData.filter(e => e.time !== undefined).map((entry, i) => {
                                console.log({ cx: xScale(entry.time), cy: yScale(entry.track) + barHeight / 2, entry })
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
                                tickFormat={(row)=>trackTypes[row].label}
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
                            {tooltipData && (
                                <g>
                                    <Line
                                        from={{ x: tooltipLeft, y: 0 }}
                                        to={{ x: tooltipLeft, y: yMax }}
                                        stroke={'lightgrey'}
                                        strokeWidth={2}
                                        pointerEvents="none"
                                        strokeDasharray="5,2"
                                    />

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
                            labelMargin="0 15px 0 0"
                            labelFormat={(l) => {console.log({l,trackTypes}); return l}}
                        />
                    </div>
                    {tooltipOpen && tooltipData && (
                        <Tooltip
                            top={tooltipTop}
                            left={tooltipLeft > 0.8 * xMax ? tooltipLeft - 75 : tooltipLeft + 75}
                            style={tooltipStyles}
                        >
                            <Box gap="xsmall">
                                {tooltipData.length > 0 && tooltipData.map((e, i) => (
                                    <div key={i}>
                                        <Box direction='row' alignContent='center' align='center' justify='start' gap='xsmall'>
                                            {e.time !== undefined && (
                                                <div style={{ borderRadius: 100, width: 7, height: 7, backgroundColor: colorScale(e.event), boxShadow: '0 0 0 2px white' }}></div>)}
                                            <Text color={colorScale(e.event)} size={e.time === undefined ? 'medium' : 'small'}>
                                                {e.label}
                                            </Text>
                                        </Box>

                                        {e.time === undefined && (<div> {round(e.progress / 1000)} / {round((e.end - e.start) / 1000)} sec</div>)}
                                        {e.time !== undefined && (<div>@ {round(e.time / 1000)} sec</div>)}

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
                                <small>{formatRow(tooltipData.group)}</small>
                            </div> */}
                        </Tooltip>

                    )}
                </div>
            </>
        );
    }
);
