import React from "react";
import { Blurb } from "./Blurb";
import { Glossary } from "./Glossary";

export function getMachineInfo({ frame, primaryColor, focusItem }) {
  const content = (
    <>
      <Blurb highlight="rgb(50,50,50)">
        <h3>About this Machine</h3>
        {focusItem.properties.description}
      </Blurb>
      <Blurb highlight="rgb(50,50,50)">
        <h3>What are Machines?</h3>
        Machines are components that are able to perform create, consume, or
        modify <Glossary.Things primaryColor={primaryColor} /> in the program.
        They define specific <Glossary.Regions primaryColor={primaryColor} />{" "}
        that are used for depositing or retrieving these things.
      </Blurb>

      {frame === "safety" && (
        <Blurb highlight={primaryColor}>
          <h3 style={{ color: primaryColor }}>Safety Concerns</h3>
          Machines define a collision zone around them, which you can observe by
          toggling them on and off in the Simulator settings. Trajectories must
          avoid these zones to avoid collisions.
        </Blurb>
      )}
    </>
  );
  return content;
}
