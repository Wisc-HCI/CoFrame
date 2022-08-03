import React from "react";
// import { Tooltip } from 'antd';
import { Blurb } from "./Blurb";
import { Glossary } from "./Glossary";

export function getGripperInfo({ frame, primaryColor, focusItem }) {
  const content = (
    <>
      <Blurb highlight="rgb(50,50,50)">
        <h3>About this Gripper</h3>
        {focusItem.properties.description}
      </Blurb>
      <Blurb highlight="rgb(50,50,50)">
        <h3>What are Grippers?</h3>
        Grippers are conventionally attached to the end of a{" "}
        <Glossary.RobotAgents primaryColor={primaryColor} />
        ’s arm, and allows it to grasp or carry{" "}
        <Glossary.Things primaryColor={primaryColor} /> and{" "}
        <Glossary.Tools primaryColor={primaryColor} />. Grippers can vary in
        size and grasping method.
      </Blurb>

      {frame === "safety" && (
        <Blurb highlight={primaryColor}>
          <h3 style={{ color: primaryColor }}>Safety Concerns</h3>
          Be mindful of the direction the gripper points when moving, since this
          can present certain safety issues if the gripper moves forward quickly
          or suddenly.
        </Blurb>
      )}
      {frame === "performance" && (
        <Blurb highlight={primaryColor}>
          <h3 style={{ color: primaryColor }}>Robot Performance</h3>
          Gripper motion <Glossary.Actions primaryColor={primaryColor} /> take
          in an optional <Glossary.Things primaryColor={primaryColor} />{" "}
          parameter, which should be specified if the intent is to grasp that
          object.
        </Blurb>
      )}
    </>
  );
  return content;
}
