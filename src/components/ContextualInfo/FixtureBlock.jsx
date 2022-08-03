import React from "react";
// import { Tooltip } from 'antd';
import { Blurb } from "./Blurb";
import { Glossary } from "./Glossary";

export function getFixtureInfo({ frame, primaryColor, focusItem }) {
  return (
    <>
      <Blurb highlight="rgb(50,50,50)">
        <h3>About this Fixture</h3>
        {focusItem.properties.description}
      </Blurb>
      <Blurb highlight="rgb(50,50,50)">
        <h3>What are Fixtures?</h3>
        Fixtures are static objects related to other components in the scene.
        For example, one could put a{" "}
        <Glossary.Machines primaryColor={primaryColor} /> or{" "}
        <Glossary.Tools primaryColor={primaryColor} /> on the surface of a
        table-like fixture. While the{" "}
        <Glossary.RobotAgents primaryColor={primaryColor} /> cannot directly
        interact with fixtures, they can nevertheless collide.
      </Blurb>

      {/* {frame === 'safety' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Safety Concerns</h3>
                       
                    </Blurb>
                )}
                {frame === 'performance' && (
                    <Blurb highlight={primaryColor}>
                        <h3 style={{color:primaryColor}}>Robot Performance</h3>
                        
                    </Blurb>
                )} */}
    </>
  );
}
