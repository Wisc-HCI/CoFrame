import React, { useCallback, memo } from "react";
import { Group } from "@visx/group";
import { AxisBottom, AxisLeft } from "@visx/axis";
import { scaleBand, scaleLinear, scaleOrdinal } from "@visx/scale";
import { withTooltip, TooltipWithBounds, defaultStyles } from "@visx/tooltip";
import { AreaClosed, Line } from "@visx/shape";
import { LinearGradient } from "@visx/gradient";
import { localPoint } from "@visx/event";
import { LegendOrdinal } from "@visx/legend";
import useStore from "../stores/Store";
import { uniq } from "lodash";
import { Box, Button, Text } from "grommet";
import { strip } from "number-precision";

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
  warningThreshold,
  errorThreshold,
  normalColor,
  warningColor,
  errorColor
) => {
  if (value >= errorThreshold) {
    return errorColor;
  } else if (value < warningThreshold) {
    return normalColor;
  } else {
    return warningColor;
  }
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

const camelCaseToWords = (str) => {
  return str
    .match(/^[a-z]+|[A-Z][a-z]*/g)
    .map(function (x) {
      return x[0].toUpperCase() + x.substr(1).toLowerCase();
    })
    .join(" ");
};

const thresholdUsed = (data, keys, evaluator) =>
  data.some((d) => keys.some((k) => evaluator(d[k])));

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
    warningThreshold = 1,
    errorThreshold = 3,
    normalColor = "grey",
    warningColor = "orange",
    errorColor = "red",
  }) => {
    const primaryColor = useStore((state) => state.primaryColor);
    const keys = Object.keys(data[0]).filter((k) => k !== "x");
    const xMaxValue = Math.max(...data.map((s) => s.x));
    const yMaxValue = Math.max(
      ...data.map((s) => Math.max(...keys.map((k) => s[k])))
    );

    console.log("Data", data);

    const normThresholdUsed = warningThreshold > 0;
    const warningThresholdUsed = thresholdUsed(
      data,
      keys,
      (d) => d >= warningThreshold && d < errorThreshold
    );
    const errorThresholdUsed = thresholdUsed(
      data,
      keys,
      (d) => d >= errorThreshold
    );
    let colorDomain = [];
    let colorRange = [];
    if (normThresholdUsed) {
      colorDomain.push(0);
      colorRange.push(normalColor);
    }
    if (warningThresholdUsed) {
      colorDomain.push(warningThreshold);
      colorRange.push(warningColor);
    }
    if (errorThresholdUsed) {
      colorDomain.push(errorThreshold);
      colorRange.push(errorColor);
    }

    const xMax = width - margin.left - margin.right;
    const yMax = height - margin.top - margin.bottom;

    const xScale = scaleLinear({
      domain: [0, xMaxValue],
    });

    const yScale = scaleLinear({
      domain: [yMaxValue, 0],
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
                    {colorDomain
                      .slice()
                      .reverse()
                      .map((v, i) => (
                        <React.Fragment key={"color" + colorRange[i]}>
                          {console.log({
                            start: {
                              color: i > 0 ? colorRange[i - 1] : colorRange[i],
                              offset:
                                i > 0
                                  ? 1 -
                                    colorDomain[i - 1] /
                                      getMaxForSeries(data, k)
                                  : 0,
                            },
                            stop: {
                                color: colorRange[i],
                                offset: 1 - v / getMaxForSeries(data, k)
                            },
                          })}
                          <stop
                            key={"color-start" + colorRange[i]}
                            offset={
                              i > 0
                                ? 1 -
                                  colorDomain[i - 1] / getMaxForSeries(data, k)
                                : 0
                            }
                            stopColor={
                              i > 0 ? colorRange[i - 1] : colorRange[i]
                            }
                          />
                          <stop
                            key={"color-stop" + colorRange[i]}
                            offset={1 - v / getMaxForSeries(data, k)}
                            stopColor={colorRange[i]}
                          />
                        </React.Fragment>
                      ))}
                    {/* <stop offset={0} stopColor={errorColor} />
                    <stop
                      offset={1 - errorThreshold / getMaxForSeries(data, k)}
                      stopColor={errorColor}
                    />
                    <stop
                      offset={1 - errorThreshold / getMaxForSeries(data, k)}
                      stopColor={warningColor}
                    />
                    <stop
                      offset={1 - warningThreshold / getMaxForSeries(data, k)}
                      stopColor={warningColor}
                    />
                    <stop
                      offset={1 - warningThreshold / getMaxForSeries(data, k)}
                      stopColor={normalColor}
                    />
                    <stop offset={1} stopColor={normalColor} /> */}
                  </linearGradient>
                </defs>
                <AreaClosed
                  data={data}
                  x={(d) => xScale(d.x)}
                  y={(d) => yScale(d[k])}
                  x0={xScale(0)}
                  y0={yScale(0)}
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
                  numTicks={10}
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
                  numTicks={10}
                  label={"Time"}
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
            {errorThresholdUsed && (
              <Line
                from={{ x: xScale(0), y: yScale(errorThreshold) }}
                to={{ x: xScale(xMaxValue), y: yScale(errorThreshold) }}
                strokeWidth={1}
                stroke={errorColor}
              />
            )}
            {(warningThresholdUsed || errorThresholdUsed) && (
              <Line
                from={{ x: xScale(0), y: yScale(warningThreshold) }}
                to={{ x: xScale(xMaxValue), y: yScale(warningThreshold) }}
                strokeWidth={1}
                stroke={warningColor}
              />
            )}

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
              labelFormat={(l) =>
                l === errorThreshold
                  ? "Error"
                  : l === warningThreshold
                  ? "Warning"
                  : "Ok"
              }
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
            <Box gap="xsmall">
              {camelCaseToWords(yAxisLabel)}
              {keys.map((key) => (
                <Box
                  key={key}
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
                      backgroundColor: getColor(
                        tooltipData[key],
                        warningThreshold,
                        errorThreshold,
                        normalColor,
                        warningColor,
                        errorColor
                      ),
                      boxShadow: "0 0 0 2px white",
                    }}
                  ></div>
                  <Text
                    color={getColor(
                      tooltipData[key],
                      warningThreshold,
                      errorThreshold,
                      normalColor,
                      warningColor,
                      errorColor
                    )}
                    size="small"
                  >
                    {camelCaseToWords(key)}
                    {" : "}
                    {strip(tooltipData[key])}
                  </Text>
                </Box>
              ))}
            </Box>
          </TooltipWithBounds>
        )}
      </div>
    );
  }
);

export default IssueGraph;
