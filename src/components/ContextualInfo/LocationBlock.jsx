import React from "react";
// import { Tooltip } from 'antd';
import { Blurb } from "./Blurb";
import { Glossary } from "./Glossary";

export function getLocationInfo({ frame, primaryColor, focusItem }) {
    // console.log('FOCUS DATA',focusData);
  const content = (
    <div>
      <Blurb highlight="rgb(50,50,50)">
        <h3>About this Location</h3>
        {focusItem.properties?.description}
      </Blurb>
      <Blurb highlight="rgb(50,50,50)">
        <h3>What are Locations?</h3>
        Locations are meaningful positions in the scene. For example, they can
        be used to define goals for placing or picking up{" "}
        <Glossary.Things primaryColor={primaryColor} />, or specifying starting
        or ending positions for the robot.
      </Blurb>

      {frame === "safety" && (
        <Blurb highlight={primaryColor}>
          <h3 style={{ color: primaryColor }}>Safety Concerns</h3>
          Pay special attention to placing locations around the occupancy zone
          of the human, since this is more likely to result in undesirable
          conflicts between the human and the robot.
        </Blurb>
      )}
      {frame === "performance" && (
        <Blurb highlight={primaryColor}>
          <h3 style={{ color: primaryColor }}>Robot Performance</h3>
          Pay special attention to the distances between start and end locations
          when creating trajectories. When spaced out far between one another,
          you may have less control over the way that the robot moves between
          the two locations, resulting in suboptimal results. Consider adding
          waypoints between them for greater control.
        </Blurb>
      )}
    </div>
  );
  return content;
}
