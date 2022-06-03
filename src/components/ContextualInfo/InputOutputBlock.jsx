import React from "react";
import { Blurb } from "./Blurb";
import { Glossary } from "./Glossary";

export function getInputOutputInfo({ frame, primaryColor, focusItem }) {
  return (
    <div>
      <Blurb highlight="rgb(50,50,50)">
        <h3>About this Input/Output</h3>
        {focusItem.properties.description}
      </Blurb>
      <Blurb highlight="rgb(50,50,50)">
        <h3>What are Inputs/Outputs?</h3>
        Inputs and Outputs specify things that can be used as components of the{" "}
        <Glossary.Processes primaryColor={primaryColor} />, frequently with
        additional information specifying how they are arranged.
      </Blurb>

      {frame === "quality" && (
        <Blurb highlight={primaryColor}>
          <h3 style={{ color: primaryColor }}>Safety Concerns</h3>
          <Glossary.Processes primaryColor={primaryColor} /> take time to
          execute, so if you start a process, you won’t be able to use their
          outputs until the process is finished and the time is up.
          Additionally, inputs cannot be used after the process has started.
        </Blurb>
      )}
      {/* {frame === 'performance' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Robot Performance</h3>
                        
                    </Blurb>
                )} */}
    </div>
  );
}
