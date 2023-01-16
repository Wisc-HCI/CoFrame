import React from "react";
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
            thresholds={focusItem.graphData.thresholds}
            units={focusItem.graphData.units}
            decimals={focusItem.graphData.decimals}
            isTimeseries={focusItem.graphData.isTimeseries}
          />
        )}
      </ParentSize>
    </div>
  );
}