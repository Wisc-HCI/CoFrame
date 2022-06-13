import React from "react";
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  Legend,
  ResponsiveContainer,
  Label,
} from "recharts";
import ParentSize from "@visx/responsive/lib/components/ParentSize";
import IssueGraph from "../IssueGraph";

export function getPlotInfo({ focusItem }) {

  return (
    <div
      style={{
        width: "100%",
        height: "200px",
        fill: "white",
      }}
    >
      {focusItem.graphData.title !== "" && (
        <h3 style={{ textAlign: "center" }}>{focusItem.graphData.title}</h3>
      )}
      <ParentSize>
        {({ width, height }) => (
          <IssueGraph
          width={width}
          height={height - 10}
          data={focusItem.graphData.series}
          xAxisLabel={focusItem.graphData.xAxisLabel}
          yAxisLabel={focusItem.graphData.yAxisLabel}
        //   warningThreshold={focusItem.graphData.warningThreshold}
        //   errorThreshold={focusItem.graphData.errorThreshold}
        //   warningColor={focusItem.graphData.warningColor}
        //   errorColor={focusItem.graphData.errorColor}
        />
        )}
      </ParentSize>
      {/* <ResponsiveContainer width="99%" aspect={3}>
                <LineChart data={focusItem.graphData.series}>
                    <CartesianGrid strokeDasharray="3 3"/>
                    <XAxis dataKey='x' interval={intervalLength}>
                        <Label value={focusItem.graphData.xAxisLabel} offset={-2} position="insideBottom" />
                    </XAxis>
                    <YAxis label={{ value: focusItem.graphData.yAxisLabel, angle: -90, position: 'insideBottomLeft'}}/>
                    {Object.keys(focusItem.graphData.series[0]).filter(entry=>entry!=='x').map((entry, idx) => {
                        return <Line type="linear" dataKey={entry} key={entry} stroke={colors[idx]} dot={false}/>
                    })}
                    <Tooltip contentStyle={{backgroundColor: '#141414'}} />
                    {showLegend && <Legend verticalAlign="top" layout="horizontal" align="right" />}
                </LineChart>
            </ResponsiveContainer> */}
    </div>
  );
}
