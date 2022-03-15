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
import { Box } from "grommet";
import { STATUS } from "../stores/Constants";

const purple1 = "#6c5efb";
const purple2 = "#c998ff";
const purple3 = "#a44afe";
const purple4 = "#9249ff";
export const background = "#eaedff";
const defaultMargin = { top: 40, left: 50, right: 40, bottom: 50 };

const tooltipStyles = {
    ...defaultStyles,
    minWidth: 60,
    backgroundColor: "rgba(0,0,0,0.9)",
    color: "white",
    padding: 5
};

const round = (num) => Math.round(num * 10) / 10

// const data = cityTemperature.slice(0, 12);

const formatRow = (row) => row.charAt(0).toUpperCase() + row.slice(1);

// scales
// const xScale = scaleLinear({
//   domain: [0, Math.max(...temperatureTotals)],
//   nice: true
// });
// const dateScale = scaleBand({
//   domain: data.map(getDate),
//   padding: 0.2
// });
// const colorScale = scaleOrdinal({
//   domain: keys,
//   range: [purple1, purple2, purple3]
// });

let tooltipTimeout;

const getGroup = (d) => d.group;

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

        const [stepData, errorType] = useStore(state => {
            let activeStep = [];
            let errorType = null;
            state.focus.some(f=>{
                if (state.programData[f]?.properties?.status === STATUS.VALID) {
                    activeStep = state.programData[f].properties.steps
                    return true
                } else { return false}
            })
            return [activeStep,]
        })
        console.log(stepData)

        const primaryColor = useStore(state => state.primaryColor)

        const data = {
            groups: ["hat", "boot", "scarf"],
            eventTypes: {
                wear: { label: "Wear", color: purple1 },
                make: { label: "Make", color: purple2 },
                bake: { label: "Bake", color: purple3 },
                knit: { label: "Knit", color: purple4 }
            },
            entries: [
                { group: "hat", event: "knit", label: "Knit Hat", start: 0, end: 4 },
                { group: "hat", event: "wear", label: "Wear Hat", start: 5, end: 8 },
                { group: "boot", event: "make", label: "Make Boot", start: 1, end: 3 },
                {
                    group: "scarf",
                    event: "bake",
                    label: "Bake Scarf",
                    start: 3,
                    end: 4
                },
                {
                    group: "scarf",
                    event: "wear",
                    label: "Wear Scarf",
                    start: 6,
                    end: 14
                }
            ]
        };

        const lastEnd = Math.max(...data.entries.map((e) => e.end));

        const yScale = scaleBand({
            domain: data.entries.map(getGroup),
            padding: 0.2
        });
        const colorScale = scaleOrdinal({
            domain: Object.keys(data.eventTypes),
            range: Object.keys(data.eventTypes).map((e) => data.eventTypes[e].color)
        });

        const xScale = scaleLinear({
            domain: [0, lastEnd],
            nice: true
        });

        const barHeight = 0.8 * yMax / uniq(data.entries.map(getGroup)).length

        xScale.rangeRound([0, xMax]);
        yScale.rangeRound([0, yMax]);

        // console.log(xScale);
        // const xMax = data.entries.map(e=>e.end);

        const handleTooltip = useCallback(
            (event) => {
                const { x, y } = localPoint(event) || { x: -10, y: -10 };
                const x0 = xScale.invert(x - defaultMargin.left);
                //   const index = bisectDate(stock, x0, 1);
                //   const y0 = yScale.invert(y-defaultMargin.top);
                // console.log(x0)
                const d = data.entries.filter(e => e.start <= x0 && e.end >= x0).map(e => ({ ...e, progress: x0 - e.start }))
                // console.log(d)
                //   const d0 = stock[index - 1];
                //   const d1 = stock[index];
                //   let d = d0;
                //   if (d1 && getDate(d1)) {
                //     d = x0.valueOf() - getDate(d0).valueOf() > getDate(d1).valueOf() - x0.valueOf() ? d1 : d0;
                //   }
                showTooltip({
                    tooltipData: d,
                    tooltipLeft: x - defaultMargin.left,
                    tooltipTop: y - defaultMargin.top,
                });
            },
            [showTooltip, yScale, xScale, data],
        );


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

                            {data.entries.map((entry, i) => {
                                return (
                                    <g key={i}>
                                        <rect
                                            rx={5}
                                            x={xScale(entry.start)}
                                            y={yScale(entry.group)}
                                            width={xScale(entry.end - entry.start)}
                                            height={barHeight}
                                            fill={colorScale(entry.event)}
                                            
                                        />
                                        <text
                                            fontSize={12}
                                            fill='white'
                                            x={xScale(entry.start)+20}
                                            y={yScale(entry.group)+0.6*barHeight}
                                        >
                                            {entry.label}
                                        </text>
                                    </g>

                                );
                            })}
                            <AxisLeft
                                // hideAxisLine
                                hideTicks
                                scale={yScale}
                                tickFormat={formatRow}
                                stroke={axisColor}
                                tickStroke={axisColor}
                                tickLabelProps={() => ({
                                    fill: axisColor,
                                    fontSize: 11,
                                    textAnchor: "end",
                                    dy: "0.33em"
                                })}
                            />
                            <AxisBottom
                                top={yMax}
                                scale={xScale}
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
                            labelFormat={(l) => data.eventTypes[l].label}
                        />
                    </div>
                    {tooltipOpen && tooltipData && (
                        <Tooltip
                            top={tooltipTop}
                            left={tooltipLeft > 0.8 * xMax ? tooltipLeft - 55 : tooltipLeft + 55}
                            style={tooltipStyles}
                        >
                            <Box gap="xsmall">
                                {tooltipData.length > 0 && tooltipData.map((e, i) => (
                                    <div key={i}>
                                        <strong style={{ color: colorScale(e.event) }}>{e.label}</strong>
                                        <div> {round(e.progress)} / {e.end - e.start} sec</div>

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
