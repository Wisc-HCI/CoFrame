import React, { useCallback } from "react";
import { Group } from "@visx/group";
import { AxisBottom, AxisLeft } from "@visx/axis";
import { scaleLinear, scaleOrdinal } from "@visx/scale";
import { withTooltip, TooltipWithBounds, defaultStyles } from "@visx/tooltip";
import { AreaClosed, Line } from "@visx/shape";
import { LinearGradient } from "@visx/gradient";
import { localPoint } from "@visx/event";
import { LegendOrdinal } from "@visx/legend";
import useStore from "../stores/Store";
// import { uniq } from "lodash";
// import { Box, Text } from "grommet";
import { strip } from "number-precision";
import { Stack, Typography } from "@mui/material";

export const background = "#eaedff";
const defaultMargin = { top: 20, left: 40, right: 10, bottom: 20 };

const tooltipStyles = {
  ...defaultStyles,
  minWidth: 60,
  backgroundColor: "rgba(0,0,0,0.9)",
  color: "white",
  padding: 5,
};

const getColor = (
  value,
  thresholds
) => {
  let color = 'Label';
  thresholds.forEach(t=>{
    if (value >= t.range[0] && value < t.range[1]) {
      color = t.color;
    }
  })
  return color
};

const findClosestData = (series, x) => {
  let closestDatum = null;
  let closestDist = Number.POSITIVE_INFINITY;
  series.some((datum) => {
    const datumDist = Math.abs(datum.x - x);
    if (datumDist <= closestDist) {
      closestDatum = datum;
      closestDist = datumDist;
      return false;
    } else {
      return true;
    }
  });
  return closestDatum;
};

const camelCaseToWords = (str) => {
  return str
    .match(/^[a-z]+|[A-Z][a-z]*/g)
    .map(function (x) {
      return x[0].toUpperCase() + x.substr(1).toLowerCase();
    })
    .join(" ");
};

const axisColor = "white";

const formatKey = (key) => key.replace(/ /g, "-");

const getMaxForSeries = (series, key) => Math.max(...series.map((s) => s[key]));

const IssueGraph = withTooltip(
  ({
    width,
    height,
    data,
    xAxisLabel,
    yAxisLabel,
    margin = defaultMargin,
    tooltipOpen,
    tooltipLeft,
    tooltipTop,
    tooltipData,
    hideTooltip,
    showTooltip,
    thresholds = [{range: ["MIN","MAX"],color:'grey',label:'Label'}],
    units = '',
    decimals = 2
  }) => {

    console.log('ISSUE GRAPH DATA',data);

    const primaryColor = useStore((state) => state.primaryColor);
    const keys = Object.keys(data[0]).filter((k) => k !== "x");
    const xMaxValue = Math.max(...data.map((s) => s.x));
    const yMaxValue = Math.max(
      ...data.map((s) => Math.max(...keys.map((k) => s[k])))
    );
    const yMinValue = Math.min(
      ...data.map((s) => Math.min(...keys.map((k) => s[k])))
    );

    const filledThresholds = thresholds.map((t) => ({
      range: [
        t.range[0] === "MIN"
          ? yMinValue
          : t.range[0] === "MAX"
          ? yMaxValue
          : t.range[0],
        t.range[1] === "MIN"
          ? yMinValue
          : t.range[1] === "MAX"
          ? yMaxValue
          : t.range[1],
      ],
      color: t.color,
    }));

    const colorDomain = filledThresholds.map((t) => t.range[0]);
    const colorRange = filledThresholds.map((t) => t.color);

    const xMax = width - margin.left - margin.right;
    const yMax = height - margin.top - margin.bottom;

    const xScale = scaleLinear({
      domain: [0, xMaxValue],
    });

    const yScale = scaleLinear({
      domain: [yMaxValue, yMinValue],
    });

    const colorScale = scaleOrdinal({
      domain: colorDomain,
      range: colorRange,
    });

    xScale.rangeRound([0, xMax]);
    yScale.rangeRound([0, yMax]);

    const handleTooltip = useCallback(
      (event, inside) => {
        const { x, y } = localPoint(event) || { x: -10, y: -10 };
        const x0 = xScale.invert(x - defaultMargin.left);
        const d = findClosestData(data, x0);
        console.log("tooltipdata", d);
        if (inside) {
          // pause();
          // reset(x0/1000);
          showTooltip({
            tooltipData: d,
            tooltipLeft: xScale(d.x),
            tooltipTop: y - defaultMargin.top,
          });
        } else {
          // play();
          hideTooltip();
        }
      },
      [showTooltip, hideTooltip, xScale, data]
    );

    const getStops = (maxValue) => {
      let stops = [];
      filledThresholds
        .slice()
        .reverse()
        .forEach((t) => {
          const o1 = 1 - t.range[0] / maxValue;
          const o2 = 1 - t.range[1] / maxValue;

          if (o2 >= 0) {
            stops.push(
              <stop key={`${t.color}-stop`} offset={o2} stopColor={t.color} />
            );
          }
          if (o1 >= 0) {
            stops.push(
              <stop key={`${t.color}-start`} offset={o1} stopColor={t.color} />
            );
          }
        });
      return stops;
    };

    return width < 10 && height < 40 && data.length > 0 ? null : (
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
            <LinearGradient
              id="area-background-gradient-issue"
              from="black"
              to="#222222"
            />
            {console.log({ tooltipOpen, tooltipData })}
            <rect
              x={0}
              y={0}
              width={width - defaultMargin.left - defaultMargin.right}
              height={
                height - defaultMargin.top - defaultMargin.bottom > 0
                  ? height - defaultMargin.top - defaultMargin.bottom
                  : 0
              }
              fill="url(#area-background-gradient-issue)"
              // fill='black'
            />
            {keys.map((k) => (
              <Group key={k} opacity={0.5}>
                {/* {console.log({id: `area-gradient-${formatKey(k)}`})} */}
                <defs>
                  <linearGradient
                    id={`area-gradient-issue-${formatKey(k)}`}
                    x1="0"
                    x2="0"
                    y1="0"
                    y2="1"
                  >
                    {getStops(getMaxForSeries(data, k))}
                  </linearGradient>
                </defs>
                <AreaClosed
                  data={data}
                  x={(d) => xScale(d.x)}
                  y={(d) => yScale(d[k])}
                  x0={xScale(0)}
                  y0={yScale(yMinValue)}
                  strokeWidth={2}
                  stroke={`url(#area-gradient-issue-${formatKey(k)})`}
                  fill={`url(#area-gradient-issue-${formatKey(k)})`}
                  fillOpacity={0.25}
                  // curve={curveMonotoneX}
                />
                <AxisLeft
                  // hideAxisLine
                  scale={yScale}
                  label={yAxisLabel}
                  // tickFormat={(row) => trackTypes[row].label}
                  stroke={axisColor}
                  // numTicks={10}
                  tickStroke={axisColor}
                  tickLabelProps={() => ({
                    fill: axisColor,
                    fontSize: 11,
                    textAnchor: "end",
                    dy: "0.33em",
                  })}
                />
                <AxisBottom
                  top={yMax}
                  scale={xScale}
                  // numTicks={10}
                  label={xAxisLabel}
                  stroke={axisColor}
                  tickStroke={axisColor}
                  tickLabelProps={() => ({
                    fill: axisColor,
                    fontSize: 11,
                    textAnchor: "middle",
                  })}
                />
              </Group>
            ))}
            {filledThresholds.map((t) => (
              <Line
                key={`${t.color}-line`}
                from={{ x: xScale(0), y: yScale(t.range[0]) }}
                to={{ x: xScale(xMaxValue), y: yScale(t.range[0]) }}
                strokeWidth={1}
                stroke={t.color}
              />
            ))}

            {tooltipData && (
              <g>
                <Line
                  from={{ x: tooltipLeft, y: 0 }}
                  to={{ x: tooltipLeft, y: yMax }}
                  stroke={"lightgrey"}
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
            fontSize: "14px",
          }}
        >
          {colorRange.length > 1 && (
            <LegendOrdinal
              labelFormat={(l) => {
                let label = 'Label';
                filledThresholds.forEach(t=>{
                  if (l >= t.range[0] && l < t.range[1]) {
                    label = t.label;
                  }
                })
                return label
              }}
              scale={colorScale}
              direction="row"
              labelMargin="0 15px 0 0"
            />
          )}
        </div>
        {tooltipOpen && tooltipData && (
          <TooltipWithBounds
            top={tooltipTop}
            left={tooltipLeft + 30}
            style={tooltipStyles}
          >
            <Stack spacing={0.5}>
              {camelCaseToWords(yAxisLabel)}
              {keys.map((key) => (
                <Stack
                  key={key}
                  direction="row"
                  alignContent="center"
                  align="center"
                  justify="start"
                  spacing={0.5}
                >
                  <div
                    style={{
                      borderRadius: 100,
                      width: 7,
                      height: 7,
                      backgroundColor: getColor(tooltipData[key],filledThresholds),
                      boxShadow: "0 0 0 2px white",
                    }}
                  ></div>
                  <Typography
                    color={getColor(tooltipData[key],filledThresholds)}
                    size="small"
                  >
                    {camelCaseToWords(key)}
                    {" : "}
                    {strip(tooltipData[key].toFixed(decimals))}
                    {' '}
                    {units}
                  </Typography>
                </Stack>
              ))}
            </Stack>
          </TooltipWithBounds>
        )}
      </div>
    );
  }
);

export default IssueGraph;
