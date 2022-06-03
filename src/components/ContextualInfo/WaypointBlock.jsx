import React from "react";
import { Blurb } from "./Blurb";
import { Glossary } from "./Glossary";

export function getWaypointInfo({ frame, primaryColor, focusItem }) {
  const content = (
    <div>
      <Blurb highlight="rgb(50,50,50)">
        <h3>About this Waypoint</h3>
        {focusItem.properties?.description}
      </Blurb>
      <Blurb highlight="rgb(50,50,50)">
        <h3>What are Waypoints?</h3>
        Waypoints are positions and orientations that are used as parts of{" "}
        <Glossary.Trajectories primaryColor={primaryColor} />, and unlike
        Locations, do not have inherent meaning other than to allow greater
        specificity of the manner with which the robot moves between a pair of
        locations.
      </Blurb>

      {frame === "safety" && (
        <Blurb highlight={primaryColor}>
          <h3 style={{ color: primaryColor }}>Safety Concerns</h3>
          Pay special attention to placing waypoints around the occupancy zone
          of the human, since this is more likely to result in undesirable
          conflicts between the human and the robot.
        </Blurb>
      )}
      {frame === "performance" && (
        <Blurb highlight={primaryColor}>
          <h3 style={{ color: primaryColor }}>Robot Performance</h3>
          Pay special attention to the sequences of waypoints and where they are
          relative to one another within a trajectory. Longer trajectories take
          longer to execute and can contribute to greater space usage.
        </Blurb>
      )}
    </div>
  );
  return content;
}
