import React from "react";
// import { Tooltip } from 'antd';
import { Blurb } from "./Blurb";
import { Glossary } from "./Glossary";
import { getPlotInfo } from "./Plots";

export function getIssueInfo({ frame, primaryColor, focusItem }) {
  console.log("FOCUS ISSUE", focusItem);
  return (
    <div>
      <Blurb highlight={primaryColor}>
        <h3>{focusItem.title}</h3>
        {focusItem.description}
      </Blurb>
      {focusItem.graphData && !focusItem.graphData.isTimeseries && <Blurb highlight="rgb(50,50,50)">{getPlotInfo({focusItem})}</Blurb>}
    </div>
  );
}
